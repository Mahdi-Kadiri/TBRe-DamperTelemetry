function results = damper_channel_health(csvPath)
% DAMPER_CHANNEL_HEALTH  Automated damper-channel health check for TBRe AiM logs.
%
% Detects non-functional or contaminated damper position channels using three
% tests, each validated against real TBRe25 endurance data (one known-healthy
% file: FSG25 endurance a_3780; one known-failed file: FSUK endurance a_3382,
% rear pot failure confirmed by team):
%
%   T1  FLATLINE FRACTION   fraction of consecutive moving samples with zero
%                           change. Healthy <= 0.011 (worst 60 s window),
%                           hard-failed channel 0.994. Threshold 0.20.
%                           Catches: open-circuit / stuck channels.
%
%   T2  ROLL RESPONSE       |Pearson r| between the band-passed (0.1-3 Hz)
%                           damper signal and band-passed lateral acceleration,
%                           moving samples only. A functioning damper channel
%                           on a circuit MUST respond to roll. Healthy windowed
%                           minimum 0.59; faulty windowed maximum 0.19 across
%                           four distinct fault modes. Threshold 0.40.
%                           PRIMARY TEST - the only one that separates all
%                           observed fault modes with margin, including
%                           brake-pressure contamination that passes T1/T3.
%                           Only run if the accelerometer axis self-check
%                           passes (see below).
%
%   T3  LOW-FREQ FRACTION   fraction of PSD power (Welch, 512-pt, Hann, 50%
%                           overlap) between 0.1-1 Hz relative to >0.1 Hz
%                           total. Healthy file-level 0.94 on all corners;
%                           faulty 0.25-0.82. Threshold 0.85. FILE-LEVEL
%                           SECONDARY ONLY: in 60 s windows the healthy and
%                           contaminated distributions overlap (healthy min
%                           0.89 vs contaminated max 0.92), so this test
%                           supports but never overrides T2.
%
% ACCELEROMETER AXIS SELF-CHECK
%   The logger's accelerometer channel names are wrong on this car
%   (verified previously and re-verified here): 'InlineAcc' is lateral,
%   'LateralAcc' is longitudinal sign-inverted. Rather than trust names,
%   the tool identifies the lateral channel from kinematics: the true
%   lateral channel must correlate with v * yaw_rate (centripetal
%   acceleration; scale-invariant so yaw-rate units don't matter).
%   Requires |r| >= 0.80 or T2 is disabled with a warning.
%
% POST-FLAG CHARACTERISATION (diagnosis, not detection):
%   - Dropout events: short excursions from a 1 s rolling median, with
%     first-occurrence time (localises intermittent-failure onset).
%   - Dominant-correlate attribution: which of {lateral acc, longitudinal
%     acc, front brake pressure, throttle} the channel best follows.
%     A damper channel whose best correlate is brake pressure is not
%     measuring suspension.
%
% KNOWN LIMITATIONS (do not remove from README):
%   - A partial-gain fault (e.g. channel at 50% amplitude) passes ALL three
%     tests: T2 is amplitude-invariant by construction. Amplitude tests were
%     rejected because a real observed fault (intermittent RL) had normal
%     amplitude. Verified by synthetic injection - see validate_channel_health.m
%   - T2 needs lateral excitation. On a straight-line or low-lateral run it
%     will degrade; the tool reports lateral-acceleration RMS so you can judge.
%   - 20 Hz logging: content above 10 Hz Nyquist is invisible and, absent
%     documented anti-alias filtering in the AiM logger, may fold into the
%     analysis band. All spectral statements carry that caveat.
%   - Validated against ONE healthy and ONE failed session (n=8 channels,
%     4 fault instances in 3 modes) plus synthetic injection. Not a
%     production-qualified detector.
%
% USAGE:  damper_channel_health                    (file picker)
%         damper_channel_health('path/to/log.csv')
%
% Self-contained: no toolboxes required (own CSV reader, filter, Welch PSD).
% Runs in MATLAB and GNU Octave.

% ── CONFIG ────────────────────────────────────────────────────────────────
THR_FLAT     = 0.20;   % T1 threshold  (healthy max 0.011 | hard fault 0.994)
THR_ROLL_R   = 0.40;   % T2 threshold  (healthy win-min 0.59 | fault win-max 0.19)
THR_LF       = 0.85;   % T3 threshold  (healthy 0.94 | fault 0.25-0.82) file-level
THR_AXIS     = 0.80;   % min |r| of lateral channel vs v*yaw to trust T2
MOVE_MPS     = 3.0;    % moving gate on 'speed' channel [m/s]
BAND         = [0.1 3.0];  % analysis band [Hz]
DROP_FRACRNG = 0.30;   % dropout: deviation from 1 s rolling median > 30% of
                       % channel's moving p99-p1 range, duration <= 1 s
SAVE_PLOTS   = true;

% ── LOAD ─────────────────────────────────────────────────────────────────
if nargin < 1 || isempty(csvPath)
    [fn, fp] = uigetfile({'*.csv','AiM CSV logs'}, 'Select log file');
    if isequal(fn,0), results = []; return; end
    csvPath = fullfile(fp, fn);
end
[T, names] = local_readcsv(csvPath);
[~, base]  = fileparts(csvPath);
getcol = @(nm) T(:, strcmp(names, nm));

t  = getcol('Time');
fs = 1/median(diff(t));
v  = getcol('speed');                       % m/s (verified; vCar_GPS is km/h)
mov = v > MOVE_MPS;

% damper channels: RAW ADC stream preferred. The calibrated FLDPS-style
% stream in the FSG25 file sample-holds 21% of samples with stalls up to
% 5.7 s while moving (CAN gaps padded by the logger) - it would false-fire
% T1 and corrupt every metric. Raw S-channels show <0.7% repeats.
rawNames = {'SFrontLeftDampe','SFrontRightDamp','SRearLeftDamper','SRearRightDampe'};
calNames = {'FLDPS','FRDPS','RLDPS','RRDPS'};
hasRaw = all(ismember(rawNames, names));
hasCal = all(ismember(calNames, names));
if hasRaw
    dnames = rawNames;
elseif hasCal
    dnames = calNames;
    fprintf(['WARNING: only calibrated channels present. These are known to\n' ...
             'sample-hold; T1 flatline results are not trustworthy on them.\n']);
else
    error('No recognised damper channels in file.');
end
corner = {'FL','FR','RL','RR'};
D = zeros(numel(t), 4);
for k = 1:4, D(:,k) = getcol(dnames{k}); end

fprintf('\n=== DAMPER CHANNEL HEALTH CHECK ===\n');
fprintf('File: %s\n', base);
fprintf('%d samples @ %.0f Hz, %.1f min, moving fraction %.2f (gate %.1f m/s)\n', ...
        numel(t), fs, (t(end)-t(1))/60, mean(mov), MOVE_MPS);

% ── ACCELEROMETER AXIS SELF-CHECK ────────────────────────────────────────
yaw  = getcol('YawRate');
dvdt = gradient(v) * fs;
vr   = v .* yaw;                            % centripetal proxy (units cancel in r)
candNames = {'InlineAcc','LateralAcc'};
rLat = zeros(1,2); rLon = zeros(1,2);
for c = 1:2
    a = getcol(candNames{c});
    rLat(c) = corr2v(a(mov), vr(mov));
    rLon(c) = corr2v(a(mov), dvdt(mov));
end
[bestR, iLat] = max(abs(rLat));
latAcc  = getcol(candNames{iLat});
iLon    = 3 - iLat;
lonAcc  = getcol(candNames{iLon}) * sign(rLon(iLon));  % sign-corrected longitudinal
axisOK  = bestR >= THR_AXIS;
fprintf(['Axis self-check: lateral = %s (|r|=%.3f vs v*yawrate), ', ...
         'longitudinal = %s (r=%.3f vs dV/dt) -> %s\n'], ...
        candNames{iLat}, bestR, candNames{iLon}, rLon(iLon), ...
        tern(axisOK, 'OK', 'FAILED - T2 disabled'));
latBPtmp = bpfilt(latAcc, BAND, fs);
latRMS = std(latBPtmp(mov));
fprintf('Lateral excitation: band-passed RMS %.3f g %s\n', latRMS, ...
        tern(latRMS < 0.05, '(LOW - T2 result unreliable)', ''));

% ── BAND-PASS ALL SIGNALS ONCE ───────────────────────────────────────────
latBP = bpfilt(latAcc, BAND, fs);
lonBP = bpfilt(lonAcc, BAND, fs);
DBP   = zeros(size(D));
for k = 1:4, DBP(:,k) = bpfilt(D(:,k), BAND, fs); end

% attribution references (only those present in file)
attrNames = {'lateral acc','longitudinal acc','PBrakeFront','RThrottleCombin'};
attrSig   = {latBP, lonBP, [], []};
for j = 3:4
    nm = attrNames{j};
    if any(strcmp(names, nm)), attrSig{j} = bpfilt(getcol(nm), BAND, fs); end
end

% ── PER-CHANNEL TESTS ────────────────────────────────────────────────────
results = struct('corner',[],'channel',[],'flatFrac',[],'rollR',[],'lfFrac',[], ...
                 'fail',[],'mode',[],'nDropout',[],'tFirstDropout',[],'attr',[]);
fprintf('\n%-4s %-17s %9s %9s %9s   %s\n','','channel','T1 flat','T2 |r|','T3 LF','verdict');
fprintf('%s\n', repmat('-',1,72));
for k = 1:4
    x  = D(:,k);  xm = x(mov);
    % T1 flatline
    flatFrac = mean(diff(xm) == 0);
    % T2 roll response
    rollR = abs(corr2v(DBP(mov,k), latBP(mov)));
    % T3 LF fraction
    lfFrac = lf_fraction(xm, fs);
    % verdicts
    f1 = flatFrac > THR_FLAT;
    f2 = axisOK && (rollR < THR_ROLL_R);
    f3 = isfinite(lfFrac) && (lfFrac < THR_LF);
    fail = f1 || f2 || f3;

    % characterisation - attribution FIRST, dropout label only if no dominant
    % non-suspension correlate (dropout counts are meaningless on a channel
    % whose amplitude scale is itself corrupted by contamination)
    nDrop = 0; tDrop = NaN; attrStr = ''; rBest = 0; jBest = 1;
    if fail
        [nDrop, tDrop] = dropouts(x, t, mov, fs, DROP_FRACRNG);
        ra = -inf(1,4);
        for j = 1:4
            if ~isempty(attrSig{j}), ra(j) = abs(corr2v(DBP(mov,k), attrSig{j}(mov))); end
        end
        [rBest, jBest] = max(ra);
        attrStr = sprintf('%s (|r|=%.2f)', attrNames{jBest}, rBest);
    end
    if f1
        mode = 'HARD FAILURE (flatline/stuck)';
    elseif fail && rBest >= 0.40 && jBest ~= 1
        mode = sprintf('CONTAMINATED - not measuring suspension; best correlate: %s', attrStr);
    elseif fail && nDrop > 20
        mode = sprintf('INTERMITTENT (%d dropouts, first at t=%.1f s)', nDrop, tDrop);
    elseif fail
        mode = sprintf('NOT TRACKING SUSPENSION - no dominant correlate (best %s)', attrStr);
    else
        mode = 'healthy';
    end
    fprintf('%-4s %-17s %9.3f %9.3f %9.3f   %s%s\n', corner{k}, dnames{k}, ...
            flatFrac, rollR, lfFrac, tern(fail,'FAIL - ','PASS - '), mode);

    results(k) = struct('corner',corner{k},'channel',dnames{k},'flatFrac',flatFrac, ...
        'rollR',rollR,'lfFrac',lfFrac,'fail',fail,'mode',mode, ...
        'nDropout',nDrop,'tFirstDropout',tDrop,'attr',attrStr);
end
fprintf('%s\n', repmat('-',1,72));
fprintf(['Thresholds: T1>%.2f  T2<%.2f  T3<%.2f. T2 is primary; T3 is file-level\n' ...
         'support only. See header for validation basis and known misses.\n'], ...
        THR_FLAT, THR_ROLL_R, THR_LF);

% ── SECONDARY: calibrated-stream integrity (when both streams present) ──
if hasRaw && hasCal
    fprintf('\nCalibrated-stream (FLDPS-style) integrity vs raw, moving samples:\n');
    for k = 1:4
        xc = getcol(calNames{k}); rep = (diff(xc)==0) & mov(2:end);
        % longest stall while moving
        d = diff([0; rep(:); 0]); st = find(d==1); en = find(d==-1)-1;
        if isempty(st), mx = 0; else, mx = max(en-st+1); end
        fprintf('  %s: %4.1f%% of moving samples held, longest moving stall %.2f s\n', ...
                calNames{k}, 100*sum(rep)/sum(mov(2:end)), mx/fs);
    end
    fprintf(['  -> calibrated stream sample-holds during CAN gaps; use raw\n' ...
             '     channels for any dynamics or health analysis.\n']);
end

% ── PLOTS ────────────────────────────────────────────────────────────────
if SAVE_PLOTS
    try
        plot_summary(results, base, THR_FLAT, THR_ROLL_R, THR_LF);
        plot_timelines(t, v, D, results, base, mov);
    catch err
        fprintf('[plot skipped: %s]\n', err.message);
    end
end
end

% ══ helpers (self-contained, MATLAB + Octave) ═════════════════════════════
function [M, names] = local_readcsv(p)
    fid = fopen(p,'r'); hdr = fgetl(fid);
    names = strsplit(strtrim(hdr), ',');
    C = textscan(fid, repmat('%f',1,numel(names)), 'Delimiter',',', ...
                 'EmptyValue', NaN, 'CollectOutput', true);
    fclose(fid);
    M = C{1};
end

function y = bpfilt(x, band, fs)
    % 0-phase band-pass: 2nd-order Butterworth HP + LP biquads (RBJ),
    % forward-backward with reflective padding. Toolbox-free.
    x = double(x(:)); x(~isfinite(x)) = 0; x = x - mean(x);
    [bh, ah] = biquad(band(1), fs, 'hp');
    [bl, al] = biquad(band(2), fs, 'lp');
    np = min(numel(x)-1, round(3*fs/band(1)));
    xp = [2*x(1)-flipud(x(2:np+1)); x; 2*x(end)-flipud(x(end-np:end-1))];
    for pass = 1:2
        xp = filter(bh, ah, xp);  xp = filter(bl, al, xp);  xp = flipud(xp);
    end
    y = xp(np+1:np+numel(x));
end

function [b,a] = biquad(fc, fs, kind)
    w0 = 2*pi*fc/fs; alpha = sin(w0)/(2*0.70710678);  cw = cos(w0);
    switch kind
        case 'lp', b = [(1-cw)/2, 1-cw, (1-cw)/2];
        case 'hp', b = [(1+cw)/2, -(1+cw), (1+cw)/2];
    end
    a = [1+alpha, -2*cw, 1-alpha];
    b = b / a(1);  a = a / a(1);
end

function r = corr2v(x, y)
    x = x(:) - mean(x); y = y(:) - mean(y);
    d = sqrt(sum(x.^2) * sum(y.^2));
    if d == 0, r = 0; else, r = sum(x.*y) / d; end
end

function lf = lf_fraction(x, fs)
    % Welch PSD, 512-pt Hann, 50% overlap; power(0.1-1 Hz)/power(>0.1 Hz)
    x = x(:) - mean(x); N = 512;
    if numel(x) < N, lf = NaN; return; end
    w = 0.5*(1 - cos(2*pi*(0:N-1)'/(N-1)));
    hop = N/2; nseg = floor((numel(x)-N)/hop) + 1;
    P = zeros(N,1);
    for s = 0:nseg-1
        seg = x(s*hop+1 : s*hop+N) .* w;
        P = P + abs(fft(seg)).^2;
    end
    f = (0:N-1)' * fs/N;  half = f <= fs/2;
    P = P(half); f = f(half);
    tot = sum(P(f > 0.1));
    if tot <= 0, lf = NaN; else, lf = sum(P(f > 0.1 & f < 1.0)) / tot; end
end

function [n, tFirst] = dropouts(x, t, mov, fs, fracRng)
    xm = x(mov);
    rng = prctile2(xm, 99) - prctile2(xm, 1);
    med = rollmed(x, round(fs)+1);
    dev = abs(x - med) > fracRng*rng;
    dev = dev & mov;
    d = diff([0; dev(:); 0]);
    st = find(d==1); en = find(d==-1)-1;
    keep = (en - st + 1) <= fs;              % <= 1 s
    st = st(keep);
    n = numel(st);
    if n > 0, tFirst = t(st(1)); else, tFirst = NaN; end
end

function m = rollmed(x, w)
    n = numel(x); m = zeros(n,1); h = floor(w/2);
    for i = 1:n
        lo = max(1,i-h); hi = min(n,i+h);
        m(i) = median(x(lo:hi));
    end
end

function q = prctile2(x, p)
    x = sort(x(:)); n = numel(x);
    q = x(max(1, min(n, round(p/100*(n-1))+1)));
end

function s = tern(c, a, b), if c, s = a; else, s = b; end, end

function plot_summary(R, base, thF, thR, thL)
    bg=[0.12 0.12 0.12]; axbg=[0.15 0.15 0.15]; txt=[0.92 0.92 0.92];
    ok=[0.30 0.90 0.40]; bad=[1.00 0.45 0.45]; thc=[1.0 0.75 0.2];
    fig=figure('Visible','off','Color',bg,'Position',[50 50 1100 420]);
    set(fig,'InvertHardcopy','off');   % keep dark background in printed PNG (MATLAB)
    vals = {[R.flatFrac],[R.rollR],[R.lfFrac]};
    ttl  = {'T1 flatline fraction (fail >)', 'T2 roll response |r| (fail <)', 'T3 LF power fraction (fail <)'};
    thr  = [thF thR thL];
    for p = 1:3
        ax = subplot(1,3,p);
        set(ax,'Color',axbg,'XColor',txt,'YColor',txt,'FontSize',9); hold(ax,'on');
        for k = 1:4
            c = ok; if R(k).fail, c = bad; end
            bar(ax, k, vals{p}(k), 0.6, 'FaceColor', c, 'EdgeColor','none');
        end
        plot(ax,[0.4 4.6],[thr(p) thr(p)],'--','Color',thc,'LineWidth',1.2);
        set(ax,'XTick',1:4,'XTickLabel',{R.corner}); ylim(ax,[0 1]);
        title(ax, ttl{p}, 'Color', txt, 'FontSize', 9);
        grid(ax,'on'); set(ax,'GridColor',[0.3 0.3 0.3]);
    end
    print(fig, sprintf('%s_channel_health.png', base), '-dpng', '-r140');
    close(fig);
end

function plot_timelines(t, v, D, R, base, mov)
    bg=[0.12 0.12 0.12]; axbg=[0.15 0.15 0.15]; txt=[0.92 0.92 0.92];
    cols={[0.25 0.60 1.00],[1.00 0.65 0.10],[0.30 0.90 0.40],[1.00 0.45 0.45]};
    fig=figure('Visible','off','Color',bg,'Position',[50 50 1100 750]);
    set(fig,'InvertHardcopy','off');   % keep dark background in printed PNG (MATLAB)
    for k = 1:4
        ax = subplot(4,1,k);
        set(ax,'Color',axbg,'XColor',txt,'YColor',txt,'FontSize',8); hold(ax,'on');
        plot(ax, t, D(:,k), 'Color', cols{k}, 'LineWidth', 0.5);
        yl = get(ax,'YLim');
        title(ax, sprintf('%s  %s - %s', R(k).corner, R(k).channel, R(k).mode), ...
              'Color', tern(R(k).fail,[1 0.45 0.45],txt), 'FontSize', 9, 'Interpreter','none');
        if k==4, xlabel(ax,'Time [s]','Color',txt); end
        ylabel(ax,'position [raw counts]','Color',txt);
        grid(ax,'on'); set(ax,'GridColor',[0.3 0.3 0.3]);
        ylim(ax, yl);
    end
    print(fig, sprintf('%s_channel_timelines.png', base), '-dpng', '-r140');
    close(fig);
end

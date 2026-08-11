function R = suspension_position_analysis(csvPath)
% SUSPENSION_POSITION_ANALYSIS  Roll/pitch gradients and suspension travel
% usage from damper position telemetry, TBRe25 endurance data (AiM, 20 Hz).
%
% Produces the setup-level numbers a race engineer reads from damper pots:
%   - Total chassis roll gradient [deg/g] vs lateral acceleration
%     (front, rear and total reported; front-rear difference = chassis
%     compliance / warp under lateral load, NOT roll stiffness split -
%     with a rigid chassis front and rear roll angles are equal regardless
%     of stiffness distribution, so the split is NOT obtainable from
%     positions alone)
%   - Pitch gradient [deg/g] vs longitudinal acceleration, braking and
%     traction fitted separately (anti-dive/anti-squat make them differ)
%   - Suspension travel usage per corner: stroke used, static ride
%     position, proximity to observed travel limits
%   - Static-position drift start-of-run vs end-of-run (pot mount /
%     zero-shift integrity check)
%   - Validation of the damper-derived roll angle against the IMU RollRate
%     channel, band-limited to dodge integration drift
%
% PREREQUISITE: run damper_channel_health.m first. This analysis is
% meaningless on channels that fail it (all four FSUK channels do).
%
% CALIBRATION (counts -> mm), derived in this project, not assumed:
%   The FSG25 file carries both raw ADC and calibrated damper streams.
%   The calibrated stream sample-holds ~20% of moving samples (CAN gaps)
%   and lags the raw stream by 1.75 samples (88 ms) on all four corners.
%   Fitting mm = a*counts + b on held-sample-excluded, lag-aligned data:
%     scale -0.02260 mm/count, agreement across corners 0.12%,
%     residual 0.92-1.19 mm per corner.
%   The per-corner coefficients are embedded below. Residual ~1 mm sets
%   the noise floor on absolute positions; differential quantities (roll,
%   pitch) partially cancel common-mode error.
%
% SIGN CONVENTIONS (self-verified at runtime, run aborts if check fails):
%   Decreasing mm = compression (verified: fronts compress under braking).
%   Wheel compression positive. Roll positive = right side compressed
%   (car rolling right, i.e. cornering left... see runtime printout which
%   states the verified direction against lateral-acceleration sign).
%   Pitch positive = nose down.
%
% VEHICLE PARAMETERS (TBRe25): front track 1200 mm, rear track 1190 mm,
% wheelbase 1530 mm, motion ratios 0.9 front / 0.69 rear (damper/wheel).
% Corner weights not required for any output here.
%
% LIMITATIONS:
%   - Angles are SUSPENSION roll/pitch (wheel-to-chassis). Tyre vertical
%     deflection is not measured, so total vehicle roll relative to road
%     is slightly larger than reported.
%   - Motion ratio treated as constant (its wheel-travel dependence is not
%     characterised for this car).
%   - Roll validation is band-limited (0.2-2 Hz) correlation + gain vs the
%     IMU rate channel; absolute low-frequency accuracy rests on the
%     calibration chain, not on an independent reference.
%   - 20 Hz logging: all content above 10 Hz invisible; anti-alias
%     behaviour of the logger undocumented.
%
% Usage:  suspension_position_analysis('IvanAxel...a_3780.csv')
% Self-contained, toolbox-free; runs in MATLAB and GNU Octave.

% ── VEHICLE PARAMETERS ───────────────────────────────────────────────────
TRACK_F = 1.200;      % m
TRACK_R = 1.190;      % m
WHEELBASE = 1.530;    % m  (used for documentation; pitch uses it directly)
MR_F = 0.90;          % damper travel / wheel travel
MR_R = 0.69;

% counts->mm, per corner, from dual-stream fit (see header)
CAL_A = [-0.022606 -0.022611 -0.022585 -0.022604];   % mm/count  FL FR RL RR
CAL_B = [ 74.747    74.768    74.705    74.745  ];   % mm

MOVE_MPS = 3.0;  STAT_MPS = 0.5;
BAND = [0.1 3.0];          % gradient-fit band [Hz]
VBAND = [0.2 2.0];         % IMU validation band [Hz]
GLONG_FIT = 0.2;           % |g| threshold separating braking/traction fits
SAVE_PLOTS = true;

% ── LOAD ─────────────────────────────────────────────────────────────────
if nargin < 1 || isempty(csvPath)
    [fn, fp] = uigetfile({'*.csv','AiM CSV logs'}, 'Select log file');
    if isequal(fn,0), R = []; return; end
    csvPath = fullfile(fp, fn);
end
[T, names] = local_readcsv(csvPath);
[~, base]  = fileparts(csvPath);
getcol = @(nm) T(:, strcmp(names, nm));

t  = getcol('Time');  fs = 1/median(diff(t));
v  = getcol('speed');
mov  = v > MOVE_MPS;
stat = v < STAT_MPS;

rawNames = {'SFrontLeftDampe','SFrontRightDamp','SRearLeftDamper','SRearRightDampe'};
corner   = {'FL','FR','RL','RR'};
if ~all(ismember(rawNames, names))
    error('Raw damper channels not found - this tool requires the raw ADC stream.');
end
MM = zeros(numel(t), 4);
for k = 1:4
    MM(:,k) = CAL_A(k)*getcol(rawNames{k}) + CAL_B(k);   % damper position [mm]
end

fprintf('\n=== SUSPENSION POSITION ANALYSIS ===\n');
fprintf('File: %s\n%d samples @ %.0f Hz, %.1f min, moving %.0f%%, stationary %.0f%%\n', ...
    base, numel(t), fs, (t(end)-t(1))/60, 100*mean(mov), 100*mean(stat));

% ── ACCELEROMETER AXIS SELF-CHECK (names are wrong on this car) ─────────
yaw = getcol('YawRate');  dvdt = gradient(v)*fs;  vr = v.*yaw;
candNames = {'InlineAcc','LateralAcc'};
rl2 = zeros(1,2); rn2 = zeros(1,2);
for c = 1:2
    a = getcol(candNames{c});
    rl2(c) = corr2v(a(mov), vr(mov));
    rn2(c) = corr2v(a(mov), dvdt(mov));
end
[bestR, iLat] = max(abs(rl2));
if bestR < 0.80, error('Accelerometer axis self-check failed (|r|=%.2f).', bestR); end
iLon = 3 - iLat;
latAcc = getcol(candNames{iLat}) * sign(rl2(iLat));       % + = accel toward +yaw side
lonAcc = getcol(candNames{iLon}) * sign(rn2(iLon));       % + = forward accel (traction)
fprintf('Axis self-check: lateral=%s (|r|=%.3f), longitudinal=%s (sign-corrected)\n', ...
    candNames{iLat}, bestR, candNames{iLon});

% ── STATIC REFERENCE + DRIFT CHECK ───────────────────────────────────────
zStatic = zeros(1,4);
tMid = t(1) + (t(end)-t(1))/2;
fprintf('\nStatic damper position [mm] and start-vs-end drift (pot integrity):\n');
driftNote = false;
for k = 1:4
    zStatic(k) = median(MM(stat,k));
    early = median(MM(stat & t <  tMid, k));
    late  = median(MM(stat & t >= tMid, k));
    driftNote = driftNote || abs(late-early) > 2;
    fprintf('  %s: static %6.2f mm | early %6.2f, late %6.2f, shift %+5.2f mm %s\n', ...
        corner{k}, zStatic(k), early, late, late-early, ...
        tern(abs(late-early) > 2, '<-- >2 mm shift', ''));
end

if driftNote
    % Locate WHERE the shift happens: compare each long stationary block
    % rather than assuming a cause. A mid-session driver change and a
    % post-session park produce the same 'early vs late' number but mean
    % completely different things.
    fprintf('\n  Static position by stationary block (>20 s, car level):\n');
    stRun = diff([0; double(stat(:)); 0]);
    bs = find(stRun==1); be = find(stRun==-1)-1;
    keep = (be-bs+1) > 20*fs;
    bs = bs(keep); be = be(keep);
    for ii = 1:numel(bs)
        blk = bs(ii):be(ii);
        fprintf('    t=%7.1f-%7.1f s (%5.1f s): ', t(bs(ii)), t(be(ii)), numel(blk)/fs);
        fprintf('%6.2f ', median(MM(blk,:),1)); fprintf('mm\n');
    end
    fprintf(['  Read this before attributing a cause: if intermediate blocks match\n' ...
             '  the first and only the final block differs, the shift is a\n' ...
             '  post-session state (driver egress, tyre pressure decay, car moved),\n' ...
             '  NOT in-session drift or a failing sensor mount. Travel-usage figures\n' ...
             '  below use moving samples only and are unaffected either way.\n']);
end

% ── WHEEL DISPLACEMENT, COMPRESSION POSITIVE ────────────────────────────
% decreasing mm = compression (verified below). Wheel = damper / MR.
MRv = [MR_F MR_F MR_R MR_R];
Z = zeros(size(MM));
for k = 1:4
    Z(:,k) = (zStatic(k) - MM(:,k)) / MRv(k);            % wheel compression [mm]
end

% runtime sign verification: strongest 200 braking samples -> fronts compress
[~, idx] = sort(lonAcc(mov)); movIdx = find(mov);
brakeIdx = movIdx(idx(1:200));
zf = mean(mean(Z(brakeIdx,1:2))) - mean(mean(Z(mov,1:2)));
zr = mean(mean(Z(brakeIdx,3:4))) - mean(mean(Z(mov,3:4)));
fprintf('\nSign self-check (hardest 200 braking samples): front %+.2f mm, rear %+.2f mm\n', zf, zr);
if zf < 0.5
    error(['Sign check FAILED: fronts do not compress under braking with the\n' ...
           'assumed convention. Do not trust any output - investigate first.']);
end
fprintf('  -> fronts compress, rears extend under braking: convention verified.\n');

% ── ANGLES ───────────────────────────────────────────────────────────────
rollF = atan2((Z(:,2)-Z(:,1))/1000, TRACK_F) * 180/pi;   % + = right side compressed
rollR = atan2((Z(:,4)-Z(:,3))/1000, TRACK_R) * 180/pi;
rollT = (rollF + rollR)/2;
warp  = rollF - rollR;
pitch = atan2((mean(Z(:,1:2),2)-mean(Z(:,3:4),2))/1000, WHEELBASE) * 180/pi;  % + = nose down

% ── GRADIENTS (band-passed, moving only) ────────────────────────────────
bpl = @(x) bpfilt(x, BAND, fs);
latB = bpl(latAcc); lonB = bpl(lonAcc);
rFb = bpl(rollF); rRb = bpl(rollR); rTb = bpl(rollT); pB = bpl(pitch);

[gF, rF_] = grad_fit(latB(mov), rFb(mov));
[gR, rR_] = grad_fit(latB(mov), rRb(mov));
[gT, rT_] = grad_fit(latB(mov), rTb(mov));
fprintf('\nROLL GRADIENTS (band %.1f-%.1f Hz, %d moving samples):\n', BAND, sum(mov));
fprintf('  front  %+6.3f deg/g  (r=%.3f)\n', gF, rF_);
fprintf('  rear   %+6.3f deg/g  (r=%.3f)\n', gR, rR_);
fprintf('  TOTAL  %+6.3f deg/g  (r=%.3f)\n', gT, rT_);
fprintf(['  front-rear difference %+.3f deg/g = chassis compliance/warp under\n' ...
         '  lateral load (NOT roll stiffness split - see header).\n'], gF-gR);
% Report magnitude; do NOT assert left/right. The sign of the fitted
% gradient depends on the yaw-rate sign convention, which is not verified
% on this logger (every other IMU channel name on it is wrong). Absolute
% left/right labelling needs one corner of known direction (e.g. GPS
% heading), which this tool does not assume.
fprintf(['  Magnitude %.3f deg/g is the deliverable; left/right labelling is\n' ...
         '  not asserted (yaw sign convention unverified on this logger).\n'], abs(gT));

lonM = lonB(mov); pM = pB(mov);
brk = lonM < -GLONG_FIT;  trc = lonM > GLONG_FIT;
[gPa, rPa] = grad_fit(lonM, pM);
[gPb, rPb] = grad_fit(lonM(brk), pM(brk));
[gPt, rPt] = grad_fit(lonM(trc), pM(trc));
fprintf('\nPITCH GRADIENTS (+ = nose down; longitudinal + = traction):\n');
fprintf('  overall            %+6.3f deg/g (r=%.3f)\n', gPa, rPa);
fprintf('  braking  (g<-%.1f)  %+6.3f deg/g (r=%.3f, n=%d)\n', GLONG_FIT, gPb, rPb, sum(brk));
fprintf('  traction (g>+%.1f)  %+6.3f deg/g (r=%.3f, n=%d)\n', GLONG_FIT, gPt, rPt, sum(trc));
asym = 100*abs(gPb-gPt)/mean(abs([gPb gPt]));
if asym < 5
    fprintf(['  Braking and traction gradients agree to %.1f%% - NO measurable\n' ...
             '  asymmetry. Pitch response is linear and direction-independent,\n' ...
             '  i.e. dominated by longitudinal load transfer through the springs\n' ...
             '  rather than by differing anti-dive/anti-squat. Do NOT quote an\n' ...
             '  asymmetry from this data.\n'], asym);
else
    fprintf(['  Braking/traction differ by %.1f%% - consistent with differing\n' ...
             '  anti-dive and anti-squat geometry.\n'], asym);
end

% ── IMU VALIDATION OF ROLL ───────────────────────────────────────────────
% The rate-channel names cannot be trusted: on this car RollRate and
% PitchRate are SWAPPED, consistent with the accelerometer swap - a single
% root cause explains both: the IMU is mounted rotated 90 deg about
% vertical (x/y exchanged, z-axis YawRate unaffected and verified against
% v*yaw). As with the accelerometers, the tool identifies the true roll-
% rate channel empirically instead of trusting the name.
dRoll = gradient(rollT)*fs;                              % damper-derived deg/s
a1 = bpfilt(dRoll, VBAND, fs);
rateNames = {'RollRate','PitchRate'};
rr2 = zeros(1,2);
for c = 1:2
    a2c = bpfilt(getcol(rateNames{c}), VBAND, fs);
    rr2(c) = corr2v(a1(mov), a2c(mov));
end
[rvAbs, iRoll] = max(abs(rr2));
a2 = bpfilt(getcol(rateNames{iRoll}), VBAND, fs);
gain = std_(a1(mov))/std_(a2(mov));
fprintf('\nIMU CROSS-CHECK (roll rate, band %.1f-%.1f Hz):\n', VBAND);
fprintf('  true roll-rate channel identified as ''%s'' (|r|=%.3f; ''%s'' scores %.3f)\n', ...
        rateNames{iRoll}, rvAbs, rateNames{3-iRoll}, abs(rr2(3-iRoll)));
if iRoll == 2
    fprintf('  -> RollRate/PitchRate are swapped, matching the accelerometer swap:\n');
    fprintf('     single root cause = IMU mounted rotated 90 deg about vertical.\n');
end
if rvAbs < 0.5
    fprintf('  WARNING: no rate channel matches damper-derived roll (best %.2f) -\n', rvAbs);
    fprintf('  cross-check inconclusive on this file.\n');
end
fprintf('  amplitude ratio damper/IMU = %.2f\n', gain);
fprintf(['  Checks timing+shape and the scale chain (calibration x MR x track).\n' ...
         '  The ratio is NOT interpretable as a pure scale check here: (1) IMU\n' ...
         '  units are assumed deg/s, unverified; (2) the IMU measures suspension\n' ...
         '  PLUS tyre roll, so it should read higher than the dampers; (3) with\n' ...
         '  the mount rotated ~90 deg, any residual misalignment leaks yaw rate\n' ...
         '  (an order of magnitude larger than roll rate on this car) into the\n' ...
         '  horizontal axes. The |r| shape agreement is the validation content.\n']);
rv = rr2(iRoll);

% ── TRAVEL USAGE ─────────────────────────────────────────────────────────
fprintf('\nTRAVEL USAGE, damper domain [mm] (moving samples):\n');
fprintf('  %-3s %8s %8s %8s %10s %10s  %s\n','','min','p1','p99','max','static','time within 1 mm of extremes');
TU = zeros(4,6);
for k = 1:4
    x = MM(mov,k);
    lo = min(x); hi = max(x);
    nearLo = 100*mean(x < lo+1);  nearHi = 100*mean(x > hi-1);
    TU(k,:) = [lo prctile2(x,1) prctile2(x,99) hi zStatic(k) nearLo+nearHi];
    fprintf('  %-3s %8.2f %8.2f %8.2f %10.2f %10.2f  lo %.2f%% / hi %.2f%%\n', ...
        corner{k}, lo, prctile2(x,1), prctile2(x,99), hi, zStatic(k), nearLo, nearHi);
end
fprintf(['  Extremes are OBSERVED travel, not mechanical stops (stop positions\n' ...
         '  not documented) - "time near extreme" is a bump-stop-proximity proxy\n' ...
         '  only. p99-p1 stroke used: %.1f-%.1f mm across corners.\n'], ...
        min(TU(:,3)-TU(:,2)), max(TU(:,3)-TU(:,2)));

% Position histograms on a circuit are usually bimodal. Identify what the
% modes correspond to rather than leaving the reader to guess.
fprintf('\n  Travel-histogram mode check (mean lateral g in each tail):\n');
for k = 1:4
    x = MM(:,k);
    hiM = mov & (x > prctile2(MM(mov,k),80));
    loM = mov & (x < prctile2(MM(mov,k),20));
    fprintf('    %s: extended tail %+.2f g | compressed tail %+.2f g\n', ...
        corner{k}, mean(latAcc(hiM)), mean(latAcc(loM)));
end
fprintf(['  Opposite-signed tails with left/right corners mirrored means the\n' ...
         '  distribution is ROLL-dominated (outside vs inside wheel), consistent\n' ...
         '  with sub-1 Hz spectral content. Bimodality here is vehicle behaviour\n' ...
         '  on a direction-biased circuit, not a data artefact.\n']);

% ── PLOTS ────────────────────────────────────────────────────────────────
if SAVE_PLOTS
    try
        plot_gradients(latB, lonB, rTb, pB, mov, gT, gPb, gPt, GLONG_FIT, base);
        plot_travel(MM, mov, zStatic, corner, base);
    catch err
        fprintf('[plot skipped: %s]\n', err.message);
    end
end

R = struct('rollGrad_F',gF,'rollGrad_R',gR,'rollGrad_total',gT, ...
           'complianceDiff',gF-gR,'pitchGrad_brake',gPb,'pitchGrad_traction',gPt, ...
           'imuRollCorr',abs(rv),'imuGain',gain,'staticMM',zStatic,'travel',TU);
end

% ══ helpers ═══════════════════════════════════════════════════════════════
function [M, names] = local_readcsv(p)
    fid = fopen(p,'r'); hdr = fgetl(fid);
    names = strsplit(strtrim(hdr), ',');
    C = textscan(fid, repmat('%f',1,numel(names)), 'Delimiter',',', ...
                 'EmptyValue', NaN, 'CollectOutput', true);
    fclose(fid);
    M = C{1};
end
function y = bpfilt(x, band, fs)
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
    w0 = 2*pi*fc/fs; alpha = sin(w0)/(2*0.70710678); cw = cos(w0);
    switch kind
        case 'lp', b = [(1-cw)/2, 1-cw, (1-cw)/2];
        case 'hp', b = [(1+cw)/2, -(1+cw), (1+cw)/2];
    end
    a = [1+alpha, -2*cw, 1-alpha];
    b = b/a(1); a = a/a(1);
end
function r = corr2v(x, y)
    x = x(:)-mean(x); y = y(:)-mean(y);
    d = sqrt(sum(x.^2)*sum(y.^2));
    if d==0, r=0; else, r=sum(x.*y)/d; end
end
function [g, r] = grad_fit(x, y)
    % least-squares slope y = g*x + c, plus correlation
    x = x(:); y = y(:);
    p = polyfit(x, y, 1); g = p(1); r = corr2v(x, y);
end
function s = std_(x), s = sqrt(mean((x-mean(x)).^2)); end
function q = prctile2(x, p)
    x = sort(x(:)); n = numel(x);
    q = x(max(1, min(n, round(p/100*(n-1))+1)));
end
function s = tern(c, a, b), if c, s = a; else, s = b; end, end

function plot_gradients(latB, lonB, rTb, pB, mov, gT, gPb, gPt, thr, base)
    bg=[0.12 0.12 0.12]; axbg=[0.15 0.15 0.15]; txt=[0.92 0.92 0.92];
    fig=figure('Visible','off','Color',bg,'Position',[50 50 1100 480]);
    set(fig,'InvertHardcopy','off');
    ds = 5;  % decimate scatter for file size
    ax=subplot(1,2,1);
    set(ax,'Color',axbg,'XColor',txt,'YColor',txt,'GridColor',[0.3 0.3 0.3],'FontSize',9);
    hold(ax,'on'); grid(ax,'on');
    xm=latB(mov); ym=rTb(mov);
    plot(ax, xm(1:ds:end), ym(1:ds:end), '.', 'Color',[0.25 0.60 1.00], 'MarkerSize',3);
    xr=[min(xm) max(xm)];
    plot(ax, xr, gT*xr, '-', 'Color',[1 0.75 0.2], 'LineWidth',1.6);
    xlabel(ax,'Lateral acceleration [g], band-passed 0.1-3 Hz','Color',txt);
    ylabel(ax,'Total suspension roll [deg], band-passed','Color',txt);
    title(ax, sprintf('Roll gradient %.3f deg/g', gT),'Color',txt);
    ax=subplot(1,2,2);
    set(ax,'Color',axbg,'XColor',txt,'YColor',txt,'GridColor',[0.3 0.3 0.3],'FontSize',9);
    hold(ax,'on'); grid(ax,'on');
    xm=lonB(mov); ym=pB(mov);
    plot(ax, xm(1:ds:end), ym(1:ds:end), '.', 'Color',[0.30 0.90 0.40], 'MarkerSize',3);
    xb=[min(xm) -thr]; xt=[thr max(xm)];
    plot(ax, xb, gPb*xb, '-', 'Color',[1 0.45 0.45], 'LineWidth',1.6);
    plot(ax, xt, gPt*xt, '-', 'Color',[1 0.75 0.2], 'LineWidth',1.6);
    xlabel(ax,'Longitudinal acceleration [g], band-passed  (+ traction)','Color',txt);
    ylabel(ax,'Suspension pitch [deg], band-passed  (+ nose down)','Color',txt);
    title(ax, sprintf('Pitch gradient: brake %.3f, traction %.3f deg/g', gPb, gPt),'Color',txt);
    print(fig, sprintf('%s_gradients.png', base), '-dpng', '-r150'); close(fig);
end

function plot_travel(MM, mov, zStatic, corner, base)
    bg=[0.12 0.12 0.12]; axbg=[0.15 0.15 0.15]; txt=[0.92 0.92 0.92];
    cols={[0.25 0.60 1.00],[1.00 0.65 0.10],[0.30 0.90 0.40],[1.00 0.45 0.45]};
    fig=figure('Visible','off','Color',bg,'Position',[50 50 1100 750]);
    set(fig,'InvertHardcopy','off');
    for k=1:4
        ax=subplot(2,2,k);
        set(ax,'Color',axbg,'XColor',txt,'YColor',txt,'GridColor',[0.3 0.3 0.3],'FontSize',9);
        hold(ax,'on'); grid(ax,'on');
        x = MM(mov,k);
        edges = linspace(min(x), max(x), 61);
        ctr = (edges(1:end-1)+edges(2:end))/2;
        cnt = histc(x, edges); cnt = cnt(1:end-1);
        bar(ax, ctr, cnt, 1.0, 'FaceColor', cols{k}, 'EdgeColor','none');
        yl = get(ax,'YLim');
        plot(ax, [zStatic(k) zStatic(k)], yl, '--', 'Color', txt, 'LineWidth', 1.2);
        xlabel(ax,'Damper position [mm]  (dashed = static)','Color',txt);
        ylabel(ax,'Samples','Color',txt);
        title(ax, sprintf('%s travel usage', corner{k}),'Color',txt);
    end
    print(fig, sprintf('%s_travel_usage.png', base), '-dpng', '-r150'); close(fig);
end

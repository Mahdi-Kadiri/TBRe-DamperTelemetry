function validate_channel_health(fsgPath)
% VALIDATE_CHANNEL_HEALTH  Validation harness for damper_channel_health.
%
% Two studies, both on the known-healthy FSG25 endurance file:
%
% 1) FALSE-ALARM SWEEP: rolling 60 s windows (50% overlap, >=70% moving),
%    all three tests applied to every healthy channel in every window.
%    Reports trip count / window count per test. This is what turns
%    "stayed quiet on one file" into a quoted false-positive rate.
%
% 2) SYNTHETIC FAULT INJECTION: known faults injected into one healthy
%    channel (FL) at full-file scale; detector re-run. Faults:
%      stuck        - constant at pre-fault value              (real: FSUK RR)
%      rail-zero    - channel forced to 0
%      noise        - white noise, variance matched to healthy signal
%      partial-50   - genuine signal scaled to 50% amplitude   (EXPECTED MISS)
%      brake-xtalk  - PBrakeFront scaled to healthy variance   (real: FSUK FL/FR)
%      dropout      - 5% of moving samples pulled to 0 in 0.1-0.5 s bursts
%                                                              (real: FSUK RL)
%    partial-50 is retained BECAUSE it is missed: T2 is amplitude-invariant,
%    and amplitude tests were rejected for cause (see README). Documenting
%    the known miss is the point.
%
% Usage: validate_channel_health('IvanAxel...a_3780.csv')

if nargin < 1
    fsgPath = 'IvanAxelEnduranceFSG25_2025Car_GenericTesting_a_3780.csv';
end
THR_FLAT = 0.20; THR_ROLL = 0.40; THR_LF = 0.85;
BAND = [0.1 3.0]; MOVE = 3.0; WIN_S = 60;

[T, names] = local_readcsv(fsgPath);
getcol = @(nm) T(:, strcmp(names, nm));
t  = getcol('Time'); fs = 1/median(diff(t));
v  = getcol('speed'); mov = v > MOVE;
S  = {'SFrontLeftDampe','SFrontRightDamp','SRearLeftDamper','SRearRightDampe'};
corner = {'FL','FR','RL','RR'};
D = zeros(numel(t),4);
for k=1:4, D(:,k)=getcol(S{k}); end
latAcc = getcol('InlineAcc');            % verified lateral (axis check in main tool)
latBP  = bpfilt(latAcc, BAND, fs);
brake  = getcol('PBrakeFront');

% ── 1) FALSE-ALARM SWEEP ─────────────────────────────────────────────────
W = round(WIN_S*fs); hop = round(W/2);
trip = zeros(3,4); nwin = 0;
worst = repmat(struct('flat',0,'roll',1,'lf',1),1,4);
for i = 1:hop:(numel(t)-W)
    sl = i:i+W-1;
    if mean(mov(sl)) < 0.70, continue; end
    nwin = nwin + 1;
    m = mov(sl);
    for k = 1:4
        xw = D(sl,k); xw = xw(m);
        fl = mean(diff(xw)==0);
        db = bpfilt(D(sl,k), BAND, fs); lb = latBP(sl);
        rr = abs(corr2v(db(m), lb(m)));
        lf = lf_fraction(xw, fs);
        trip(1,k) = trip(1,k) + (fl > THR_FLAT);
        trip(2,k) = trip(2,k) + (rr < THR_ROLL);
        trip(3,k) = trip(3,k) + (isfinite(lf) && lf < THR_LF);
        worst(k).flat = max(worst(k).flat, fl);
        worst(k).roll = min(worst(k).roll, rr);
        if isfinite(lf), worst(k).lf = min(worst(k).lf, lf); end
    end
end
fprintf('\n=== FALSE-ALARM SWEEP: %d x 60 s healthy windows (%d channel-windows) ===\n', nwin, nwin*4);
fprintf('%-4s | T1 trips (worst flat) | T2 trips (worst |r|) | T3 trips (worst LF)\n','');
for k = 1:4
    fprintf('%-4s |   %2d/%2d   (%.3f)     |   %2d/%2d   (%.3f)    |   %2d/%2d   (%.3f)\n', ...
        corner{k}, trip(1,k),nwin,worst(k).flat, trip(2,k),nwin,worst(k).roll, ...
        trip(3,k),nwin,worst(k).lf);
end
fprintf('Total false positives: T1 %d, T2 %d, T3 %d across %d channel-windows.\n', ...
        sum(trip(1,:)), sum(trip(2,:)), sum(trip(3,:)), nwin*4);

% ── 2) SYNTHETIC FAULT INJECTION into FL ─────────────────────────────────
x0 = D(:,1); sd0 = std(x0(mov)); mu0 = mean(x0(mov));
rng_seed(42);
faults = {'stuck','rail-zero','noise','partial-50','brake-xtalk','dropout'};
fprintf('\n=== SYNTHETIC FAULT INJECTION (channel FL, full file) ===\n');
fprintf('%-12s %8s %8s %8s   %s\n','fault','T1 flat','T2 |r|','T3 LF','detected by');
fprintf('%s\n', repmat('-',1,64));
for fi = 1:numel(faults)
    x = x0;
    switch faults{fi}
        case 'stuck',       x(:) = mu0;
        case 'rail-zero',   x(:) = 0;
        case 'noise',       x = mu0 + sd0*randn_local(numel(x));
        case 'partial-50',  x = mu0 + 0.5*(x0 - mu0);
        case 'brake-xtalk'
            b = brake - mean(brake(mov));
            x = mu0 + b * (sd0/std(b(mov))) + 0.05*sd0*randn_local(numel(x));
        case 'dropout'
            x = x0; i = 1; n = numel(x); nd = 0; target = 0.05*sum(mov);
            while nd < target
                i = i + round((0.5+2*rand_local())*fs);
                if i > n-12, break; end
                if ~mov(i), continue; end
                L = round((0.1+0.4*rand_local())*fs);
                x(i:min(n,i+L)) = 0; nd = nd + L;
            end
    end
    xm = x(mov);
    fl = mean(diff(xm)==0);
    db = bpfilt(x, BAND, fs);
    rr = abs(corr2v(db(mov), latBP(mov)));
    lf = lf_fraction(xm, fs);
    hits = {};
    if fl > THR_FLAT, hits{end+1}='T1'; end
    if rr < THR_ROLL, hits{end+1}='T2'; end
    if isfinite(lf) && lf < THR_LF, hits{end+1}='T3'; end
    if isempty(hits), verdict = '** MISSED **'; else, verdict = strjoin(hits,','); end
    fprintf('%-12s %8.3f %8.3f %8.3f   %s\n', faults{fi}, fl, rr, lf, verdict);
end
fprintf('%s\n', repmat('-',1,64));
fprintf(['partial-50 miss is expected and documented: all three tests are\n' ...
         'amplitude-invariant or amplitude-agnostic by design (see README).\n']);
end

% ══ helpers (duplicated so each file is standalone) ═══════════════════════
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
function lf = lf_fraction(x, fs)
    x = x(:)-mean(x); N = 512;
    if numel(x) < N, lf = NaN; return; end
    w = 0.5*(1-cos(2*pi*(0:N-1)'/(N-1)));
    hop = N/2; nseg = floor((numel(x)-N)/hop)+1;
    P = zeros(N,1);
    for s = 0:nseg-1
        seg = x(s*hop+1 : s*hop+N).*w;
        P = P + abs(fft(seg)).^2;
    end
    f = (0:N-1)'*fs/N; half = f<=fs/2; P=P(half); f=f(half);
    tot = sum(P(f>0.1));
    if tot<=0, lf=NaN; else, lf = sum(P(f>0.1 & f<1.0))/tot; end
end
% minimal deterministic RNG in double precision - identical in MATLAB and
% Octave (no integer wraparound semantics involved). Quality is sufficient
% for fault injection; do not reuse for anything statistical.
function rng_seed(s), global LCG_STATE; LCG_STATE = double(s); end
function r = rand_local()
    global LCG_STATE
    LCG_STATE = mod(LCG_STATE*9301 + 49297, 233280);
    r = LCG_STATE / 233280;
end
function v = randn_local(n)
    v = zeros(n,1);
    for i = 1:2:n
        u1 = max(rand_local(),1e-12); u2 = rand_local();
        g = sqrt(-2*log(u1)); v(i) = g*cos(2*pi*u2);
        if i+1<=n, v(i+1) = g*sin(2*pi*u2); end
    end
end

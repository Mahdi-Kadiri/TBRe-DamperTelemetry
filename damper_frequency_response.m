function damper_frequency_response(csvPath)
% DAMPER_FREQUENCY_RESPONSE
% =========================================================================
% Damper frequency-content and velocity-distribution analysis, TBRe25
% endurance telemetry (AiM, 20 Hz).
%
% RUN THE CHANNEL HEALTH CHECK FIRST (damper_channel_health.m).
% This analysis is only meaningful on channels that pass it. On the FSUK
% endurance file (a_3382) ALL FOUR damper channels fail the health check
% (two rear sensor failures + brake-pressure contamination on both fronts),
% so nothing in this script should be quoted from that file.
%
% TWO ANALYSES:
%   1. FREQUENCY SPECTRUM (Welch PSD) of damper position per corner.
%   2. DAMPER VELOCITY HISTOGRAM split bump/rebound.
%
% SAMPLING LIMITATIONS (quote these with any result):
%   - 20 Hz logging -> 10 Hz Nyquist. Sprung-mass/body content (~1-2 Hz) is
%     resolvable. Unsprung/wheel-hop content (~12-18 Hz) is NOT resolvable,
%     and WITHOUT documented anti-alias filtering ahead of the 20 Hz sampler
%     it may fold (alias) into the analysis band - low-frequency content
%     cannot be certified free of folded high-frequency energy.
%   - Velocities are obtained by differentiating 20 Hz position, then
%     smoothed with a 5-sample moving mean (~2 Hz effective bandwidth).
%     Histogram values are therefore LOW-FREQUENCY damper velocity
%     components, NOT peak damper velocities. Kerb-strike velocity content
%     is far above this bandwidth. Do not quote "max bump/rebound" numbers
%     as peak damper speed.
%   - Velocities are in the DAMPER domain (sensor axis). Wheel-domain
%     velocities differ by the motion ratio (0.9 front / 0.69 rear on the
%     TBRe25 car): divide damper velocity by motion ratio for wheel domain.
%   - SIGN CONVENTION IS UNVERIFIED: '+ = bump' assumes the pot reads
%     increasing under compression, which has not been confirmed against a
%     known event on this car. Verify against a hard-braking event (fronts
%     must compress) before quoting any bump/rebound asymmetry.
%
% DATA CONTEXT: written for full endurance files (FSG25 a_3780: 59.5 min,
% 22 km, ~0.7 g RMS lateral). Energy is dominated by sub-1 Hz chassis
% motion (roll/pitch/heave following track inputs); resonance peaks sit on
% top of that. (An earlier version of this header described car-park data;
% that description was wrong for these files.)
% =========================================================================
MOVE_SPEED_THRESHOLD = 3.0;   % m/s gate
SEGMENT_LEN = 2048;           % FFT window length (power of 2)
MIN_SEG_MOVING_FRAC = 0.95;   % contiguous-segment requirement

% ── LOAD ─────────────────────────────────────────────────────────────────
if nargin < 1 || isempty(csvPath)
    [fn, fp] = uigetfile({'*.csv','AiM CSV logs'}, 'Select log file');
    if isequal(fn, 0), return; end
    csvPath = fullfile(fp, fn);
end
[fp, fnb, fne] = fileparts(csvPath); if isempty(fp), fp='.'; end
fn = [fnb fne];
fid = fopen(csvPath, 'r');
hdr = fgetl(fid); varNames = strsplit(strtrim(hdr), ',');
C = textscan(fid, repmat('%f',1,numel(varNames)), 'Delimiter',',', ...
             'EmptyValue', NaN, 'CollectOutput', true);
fclose(fid);
M = C{1};
getcol = @(nm) M(:, strcmp(varNames, nm));

t  = getcol('Time');
v  = getcol('speed');
dt = median(diff(t));
fs = 1/dt;

% RAW ADC channels preferred - the calibrated FLDPS-style stream sample-
% holds ~20% of moving samples with multi-second stalls (see channel-health
% README) and must not be used for spectral analysis.
if any(strcmp(varNames,'SFrontLeftDampe'))
    fl = getcol('SFrontLeftDampe'); fr = getcol('SFrontRightDamp');
    rl = getcol('SRearLeftDamper'); rr = getcol('SRearRightDampe');
    unitStr = 'counts (raw ADC)';
else
    fl = getcol('FLDPS'); fr = getcol('FRDPS');
    rl = getcol('RLDPS'); rr = getcol('RRDPS');
    unitStr = 'mm (calibrated stream - beware sample-hold stalls)';
end

fprintf('Loaded %d samples at %.0f Hz (Nyquist %.0f Hz), units: %s\n', ...
        numel(t), fs, fs/2, unitStr);

dampers = {'Front Left', fl; 'Front Right', fr; 'Rear Left', rl; 'Rear Right', rr};

% ── DARK THEME ───────────────────────────────────────────────────────────
bg=[0.12 0.12 0.12]; ax_bg=[0.15 0.15 0.15]; txt=[0.92 0.92 0.92];
grid_c=[0.30 0.30 0.30];
cols = {[0.25 0.60 1.00],[1.00 0.65 0.10],[0.30 0.90 0.40],[1.00 0.45 0.45]};
[~, base] = fileparts(fullfile(fp, fn));

% ── SELECT A CONTIGUOUS MOVING SEGMENT ───────────────────────────────────
% The old version took SEGMENT_LEN samples from the first moving sample
% without checking the car STAYED moving - on the FSUK file that window
% contained a stop AND straddled a sensor-failure onset. This version
% requires >=95% of the window moving, and picks the qualifying window
% with the highest mean speed (most representative running).
moving = v > MOVE_SPEED_THRESHOLD;
bestStart = NaN; bestSpeed = -inf;
for i = 1:round(SEGMENT_LEN/4):(numel(t)-SEGMENT_LEN)
    w = i:i+SEGMENT_LEN-1;
    if mean(moving(w)) >= MIN_SEG_MOVING_FRAC && mean(v(w)) > bestSpeed
        bestSpeed = mean(v(w)); bestStart = i;
    end
end
if isnan(bestStart)
    error('No contiguous %d-sample moving segment found - check the data.', SEGMENT_LEN);
end
seg = bestStart:bestStart+SEGMENT_LEN-1;
fprintf('Spectrum segment: t = %.1f-%.1f s, mean speed %.1f m/s\n', ...
        t(seg(1)), t(seg(end)), bestSpeed);

% ── ANALYSIS 1: FREQUENCY SPECTRUM ───────────────────────────────────────
fig1 = figure('Visible','off','Color',bg,'Position',[50 50 1100 750]);
set(fig1,'InvertHardcopy','off');
for d = 1:4
    sig = dampers{d,2}(seg);
    sig = sig - mean(sig,'omitnan');
    sig(~isfinite(sig)) = 0;
    win = 0.5*(1 - cos(2*pi*(0:numel(sig)-1)'/(numel(sig)-1)));   % Hann
    Y = fft(sig .* win);
    N = numel(sig);
    f = (0:N-1)*(fs/N);
    mag = abs(Y)/N;
    half = 1:floor(N/2);

    ax = subplot(2,2,d);
    set(ax,'Color',ax_bg,'XColor',txt,'YColor',txt,'GridColor',grid_c,'FontSize',9);
    hold(ax,'on'); grid(ax,'on');
    plot(ax, f(half), mag(half), 'Color',cols{d}, 'LineWidth',1.2);
    xlim(ax, [0 fs/2]);
    xlabel(ax,'Frequency [Hz]','Color',txt);
    ylabel(ax,sprintf('Amplitude [%s]', strtok(unitStr)),'Color',txt);
    title(ax, sprintf('%s - spectrum', dampers{d,1}),'Color',txt,'FontSize',10);
end
annotation(fig1,'textbox',[0.15 0.955 0.7 0.04],'String', ...
    sprintf('Damper position spectra (20 Hz logging; unverified anti-aliasing - see header)'), ...
    'Color',txt,'EdgeColor','none','HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
out1 = fullfile(fp, sprintf('%s_damper_fft.png', base));
print(fig1, out1,'-dpng','-r150'); close(fig1);

% ── ANALYSIS 2: DAMPER VELOCITY HISTOGRAM ────────────────────────────────
fig2 = figure('Visible','off','Color',bg,'Position',[50 50 1100 750]);
set(fig2,'InvertHardcopy','off');
for d = 1:4
    pos = dampers{d,2};
    posSm = movmean_local(pos, 5);
    vel = gradient(posSm) ./ dt;
    vel_m = vel(moving);

    ax = subplot(2,2,d);
    set(ax,'Color',ax_bg,'XColor',txt,'YColor',txt,'GridColor',grid_c,'FontSize',9);
    hold(ax,'on'); grid(ax,'on');
    % hist counts computed manually (works in MATLAB and Octave; 'histogram'
    % is MATLAB-only)
    edges = linspace(min(vel_m), max(vel_m), 51);
    ctr = (edges(1:end-1)+edges(2:end))/2;
    cnt = histc(vel_m, edges); cnt = cnt(1:end-1);
    bar(ax, ctr, cnt, 1.0, 'FaceColor',cols{d}, 'EdgeColor','none');
    xlabel(ax,sprintf('Damper velocity [%s/s]  (sign convention UNVERIFIED)', strtok(unitStr)),'Color',txt);
    ylabel(ax,'Count','Color',txt);
    title(ax, sprintf('%s - low-freq velocity distribution', dampers{d,1}),'Color',txt,'FontSize',10);
end
annotation(fig2,'textbox',[0.15 0.955 0.7 0.04],'String', ...
    'Damper velocity distribution - LOW-FREQUENCY COMPONENT ONLY (~2 Hz bandwidth)', ...
    'Color',txt,'EdgeColor','none','HorizontalAlignment','center','FontSize',11,'FontWeight','bold');
out2 = fullfile(fp, sprintf('%s_damper_velocity_hist.png', base));
print(fig2, out2,'-dpng','-r150'); close(fig2);

% ── CONSOLE SUMMARY ──────────────────────────────────────────────────────
fprintf('\n-- Damper low-frequency velocity summary (NOT peak damper speed) --\n');
for d = 1:4
    pos = dampers{d,2};
    vel = gradient(movmean_local(pos,5)) ./ dt;
    vel_m = vel(moving);
    pos_v = vel_m(vel_m > 0);  neg_v = vel_m(vel_m < 0);
    if isempty(pos_v) || isempty(neg_v)
        fprintf('%s: degenerate channel (one-sided or zero velocity) - run the health check.\n', dampers{d,1});
        continue
    end
    fprintf('%s: max +%.0f, max -%.0f %s/s (2 Hz-band values), +/- mean ratio %.2f\n', ...
        dampers{d,1}, max(pos_v), abs(min(neg_v)), strtok(unitStr), ...
        mean(pos_v)/abs(mean(neg_v)));
end
fprintf('\n[OK] Saved:\n  %s\n  %s\n', out1, out2);
end

function y = movmean_local(x, w)
    % moving mean, toolbox-free, NaN-tolerant
    x = x(:); x(~isfinite(x)) = 0;
    k = ones(w,1)/w;
    y = conv(x, k, 'same');
end

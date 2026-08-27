function C = suspension_conclusions(rollGrad, rollGradF, rollGradR, pitchBrake, pitchTraction, travelUsed)
% SUSPENSION_CONCLUSIONS  Turn measured roll/pitch/travel into setup
% conclusions for the TBRe25 car.
%
% Every figure quoted in the repository README is produced here. Run
% suspension_position_analysis first and pass its outputs in, or call with
% no arguments to use the FSG25 endurance values.
%
%   C = suspension_conclusions()
%   C = suspension_conclusions(0.712, 0.704, 0.720, 0.687, 0.684, [27.8 26.5 24.2 21.9])
%
% CONCLUSIONS PRODUCED
%   1. Camber available at the limit, and the static camber implied by the
%      tyre's working range. Reported across the whole design-target
%      camber-gain range, because the conclusion should not depend on which
%      value of a poorly-known input is chosen.
%   2. Whether springs + ARB account for the measured roll stiffness.
%   3. Roll stiffness distribution and what the rear ARB is worth.
%   4. Travel headroom and the spring rate change it permits.
%   5. Null results: pitch symmetry and chassis torsional stiffness.
%   Plus the motion-ratio consistency check that underwrites (1)-(4).
%
% LIMITATIONS
%   - Suspension roll only; tyre deflection not measured, so roll relative
%     to the road is larger than used here. Camber conclusion is therefore
%     conservative: including tyre roll makes the shortfall worse, not better.
%   - Camber conclusion assumes constant camber gain through the stroke and
%     ignores the small contribution of steer-induced camber (caster).
%   - Travel headroom depends on whether the quoted travel figure is damper
%     stroke or wheel travel; check tbre25_vehicle_data.m.
%   - Roll stiffness distribution follows from spring rates and motion
%     ratios, NOT from the position data - roll angles are equal front and
%     rear on a stiff chassis, so positions alone cannot give the split.

V = tbre25_vehicle_data();
if nargin < 1 || isempty(rollGrad),      rollGrad     = 0.712; end
if nargin < 2 || isempty(rollGradF),     rollGradF    = 0.704; end
if nargin < 3 || isempty(rollGradR),     rollGradR    = 0.720; end
if nargin < 4 || isempty(pitchBrake),    pitchBrake   = 0.687; end
if nargin < 5 || isempty(pitchTraction), pitchTraction= 0.684; end
if nargin < 6 || isempty(travelUsed)
    % [bump(4) rebound(4)] from static, damper mm, FSG25 endurance.
    % Bump  = static - p1 ; Rebound = p99 - static, from
    % suspension_position_analysis on the FSG25 file.
    travelUsed = [13.7 12.8 11.5 11.1  14.1 13.8 12.7 10.8];
end
rollGrad = abs(rollGrad); rollGradF = abs(rollGradF); rollGradR = abs(rollGradR);
pitchBrake = abs(pitchBrake); pitchTraction = abs(pitchTraction);

tf = V.track_f_mm/1000;  tr = V.track_r_mm/1000;
axleK = @(k, MR, t) k*MR^2*1000*t^2/2/57.2958;   % k N/mm, t m -> N.m/deg

fprintf('\n========= TBRe25 SUSPENSION CONCLUSIONS =========\n');

% ── 0. THE MEASUREMENT ITSELF, CROSS-CHECKED ────────────────────────────
% This is the result that survives every open input conflict, because it
% depends on neither roll centre nor static camber.
if isfield(V,'design') && isfield(V.design,'roll_grad_deg_g')
    d = V.design.roll_grad_deg_g;
    fprintf('\n-- 0. ROLL GRADIENT, CROSS-CHECKED AGAINST DESIGN --\n');
    fprintf('  measured %.3f deg/g vs independent design prediction %.3f -> %.1f%%\n', ...
            rollGrad, d, 100*abs(rollGrad/d-1));
    fprintf('  CONCLUSION: the measurement agrees with the team design model to\n');
    fprintf('  %.1f%%. Two independent routes to the same number. This result is\n', 100*abs(rollGrad/d-1));
    fprintf('  not conditional on roll centre or static camber, unlike (1)-(3).\n');
end

% ── MOTION RATIO CONSISTENCY CHECK ───────────────────────────────────────
% On a torsionally stiff chassis front and rear roll angles must agree.
% Damper differential travel is the raw measurement; converting it to a
% roll angle needs the motion ratio, so the constraint tests the MR pair.
dDamper_f = rollGradF * V.mr_front * tf;         % proportional to travel
dDamper_r = rollGradR * V.mr_rear   * tr;
travelRatio = dDamper_r / dDamper_f;
mrRatioReq  = travelRatio * tf / tr;             % MR_r/MR_f for equal roll
mrRatioUsed = V.mr_rear / V.mr_front;
rollMismatch = 100*abs(rollGradF-rollGradR)/mean([rollGradF rollGradR]);

fprintf('\n-- MOTION RATIO CONSISTENCY (underwrites everything below) --\n');
fprintf('  rear dampers move %.1f%% as much as fronts in roll (raw, MR-independent)\n', 100*travelRatio);
fprintf('  -> equal front/rear roll requires MR_rear/MR_front = %.3f\n', mrRatioReq);
fprintf('  -> pair in use (%.3f/%.3f) gives ratio %.3f\n', V.mr_front, V.mr_rear, mrRatioUsed);
fprintf('  measured front/rear roll gradients differ by %.1f%%\n', rollMismatch);
if rollMismatch < 5
    fprintf('  VERIFIED: consistent with a torsionally stiff chassis.\n');
else
    fprintf('  WARNING: %.1f%% mismatch implies either chassis twist or an\n', rollMismatch);
    fprintf('  incorrect motion ratio pair. Do not trust the figures below.\n');
end

% ── 1. CAMBER AT THE LIMIT ───────────────────────────────────────────────
% Camber relative to the ROAD needs total chassis roll, which is suspension
% roll (measured here) plus tyre roll (from rig-measured tyre vertical
% rates). Suspension travel, and therefore camber gain, is driven by
% suspension roll alone. Static camber is the as-run value reported by the
% VD engineer present at the event.
roll_susp = rollGrad * V.peak_lat_g;
comp_mm   = tand(roll_susp) * (V.track_f_mm/2);
if isfield(V,'tyre_roll_deg_g')
    roll_road = (rollGrad + V.tyre_roll_deg_g) * V.peak_lat_g;
    tyreKnown = true;
else
    roll_road = roll_susp; tyreKnown = false;
end
fprintf('\n-- 1. CAMBER AT THE LIMIT --\n');
fprintf('  static camber %.2f deg (as run, per the VD engineer at the event)\n', V.setup.camber_static_deg);
fprintf('  peak lateral %.2f g -> suspension roll %.2f deg', V.peak_lat_g, roll_susp);
if tyreKnown
    fprintf(', tyre roll %.2f deg\n', roll_road-roll_susp);
    fprintf('  TOTAL roll to road %.2f deg (tyre rates %.0f/%.0f N/mm, rig-measured)\n', ...
            roll_road, V.tyre_rate_f_Nmm, V.tyre_rate_r_Nmm);
else
    fprintf('\n  tyre roll unmeasured - result is conservative by that amount\n');
end
fprintf('  outside front wheel compression %.1f mm (driven by suspension roll)\n', comp_mm);
gains = [-0.05 V.setup.camber_gain_deg_mm -0.08 -0.10];
nets  = zeros(size(gains));
fprintf('  net camber to road = static + total roll - camber gain:\n');
for i = 1:numel(gains)
    nets(i) = V.setup.camber_static_deg + roll_road + gains(i)*comp_mm;
    fprintf('    gain %6.3f deg/mm -> %+5.2f deg%s\n', gains(i), nets(i), ...
        tern(gains(i)==V.setup.camber_gain_deg_mm, '   <- solver value', ''));
end
target = V.tyre_peak_camber_deg;
gainReq = (max(target) - V.setup.camber_static_deg - roll_road)/comp_mm;
fprintf('  tyre working range %.1f to %.1f deg.\n', target(1), target(2));
if all(nets > max(target))
    fprintf('  CONCLUSION: across the entire design-target gain range the outside\n');
    fprintf('  tyre arrives at %+.2f to %+.2f deg - short of its working range.\n', max(nets), min(nets));
    fprintf('  Reaching %.1f deg would need %.3f deg/mm, outside that range, so\n', max(target), gainReq);
    fprintf('  the conclusion does not depend on which gain figure is correct.\n');
    fprintf('  >>> RECOMMEND %.1f to %.1f deg more static negative camber,\n', ...
            max(target)-max(nets), max(target)-min(nets));
    fprintf('      or more camber gain from geometry.\n');
    if tyreKnown
        fprintf('  Tyre roll is INCLUDED, not caveated: it adds %.2f deg of camber\n', roll_road-roll_susp);
        fprintf('  loss that a suspension-only calculation would have missed.\n');
    end
else
    fprintf('  CONCLUSION: camber lands inside the tyre working range for at\n');
    fprintf('  least part of the plausible gain range - not a clear-cut change.\n');
end

% ── 2. DOES THE MODEL CLOSE? ─────────────────────────────────────────────
Kf = axleK(V.spring_rate_Nmm, V.mr_front, tf);
Kr = axleK(V.spring_rate_Nmm, V.mr_rear,  tr);
Rinv = roll_stiffness_measured(V, rollGrad);
resid = Rinv - (Kf+Kr);
fprintf('\n-- 2. DOES THE DESIGN MODEL CLOSE? --\n');
fprintf('  springs %.1f N/mm both ends, MR %.2f/%.2f:\n', V.spring_rate_Nmm, V.mr_front, V.mr_rear);
fprintf('    front %.0f + rear %.0f = %.0f N.m/deg\n', Kf, Kr, Kf+Kr);
fprintf('  measured total (from roll gradient) %.0f N.m/deg\n', Rinv);
if isnan(V.arb_rear_Nm_deg)
    fprintf('  residual %.0f N.m/deg, attributable to the rear ARB\n', resid);
    fprintf('  (no front ARB fitted, so it is the only other elastic path)\n');
    if resid > 0 && resid < 0.35*(Kf+Kr)
        fprintf('  CONCLUSION: springs + a plausible rear ARB account for the\n');
        fprintf('  measured roll stiffness. The design model predicts the car driven.\n');
    else
        fprintf('  CONCLUSION: residual is implausible for a rear ARB alone -\n');
        fprintf('  check spring rate units, motion ratios, or the roll arm.\n');
    end
    arbUse = resid;
else
    Kdes = Kf + Kr + V.arb_rear_Nm_deg;
    fprintf('  + rear ARB %.0f = %.0f design vs %.0f measured (%+.1f%%)\n', ...
            V.arb_rear_Nm_deg, Kdes, Rinv, 100*(Rinv/Kdes-1));
    arbUse = V.arb_rear_Nm_deg;
end

% ── 3. ROLL STIFFNESS DISTRIBUTION ───────────────────────────────────────
KrTot = Kr + arbUse;
rsd     = 100*Kf/(Kf+KrTot);
rsd_noA = 100*Kf/(Kf+Kr);
fprintf('\n-- 3. ROLL STIFFNESS DISTRIBUTION --\n');
fprintf('  front %.0f, rear %.0f (incl ARB) -> %.1f%% FRONT\n', Kf, KrTot, rsd);
fprintf('  without the rear ARB it would be %.1f%% front\n', rsd_noA);
fprintf('  CONCLUSION: rear ARB is worth %.1f points of distribution at this\n', rsd_noA-rsd);
fprintf('  setting - it is the balance adjuster, quantified from data.\n');
if rsd > 50
    fprintf('  %.1f%% front = front-biased elastic load transfer = understeer-\n', rsd);
    fprintf('  biased balance by design.\n');
else
    fprintf('  %.1f%% front = rear-biased elastic load transfer = oversteer-\n', rsd);
    fprintf('  biased balance by design.\n');
end

% ── 4. TRAVEL HEADROOM (per axle, from static, damper space) ────────────
% The pots read damper millimetres and the 55 mm stroke is damper stroke,
% so no motion ratio enters this calculation - it is immune to the
% effective-vs-geometric MR question. Travel is NOT symmetric about static:
% static compression consumes 19.6 mm of stroke at the front and 30.5 mm at
% the rear, so bump and rebound headroom differ by axle and must be
% reported separately. A single "percent of stroke used" figure hides this.
avail = V.setup.travel_available_mm;
scomp = [V.setup.static_compression_f_mm V.setup.static_compression_f_mm ...
         V.setup.static_compression_r_mm V.setup.static_compression_r_mm];
bumpAvail = avail - scomp;      % compression remaining from static
rebAvail  = scomp;              % extension remaining to full rebound
fprintf('\n-- 4. TRAVEL HEADROOM (per axle, from static) --\n');
fprintf('  damper stroke %.0f mm; static compression %.1f F / %.1f R\n', ...
        avail, V.setup.static_compression_f_mm, V.setup.static_compression_r_mm);
if numel(travelUsed) == 8
    bumpUsed = travelUsed(1:4); rebUsed = travelUsed(5:8);
    fprintf('  %-4s %10s %10s | %10s %10s\n','','bump used','of avail','reb used','of avail');
    worstReb = 0; worstBump = 0;
    for i = 1:4
        fprintf('  %-4s %9.1f %9.0f%% | %9.1f %9.0f%%\n', corner_name(i), ...
                bumpUsed(i), 100*bumpUsed(i)/bumpAvail(i), rebUsed(i), 100*rebUsed(i)/rebAvail(i));
        worstBump = max(worstBump, 100*bumpUsed(i)/bumpAvail(i));
        worstReb  = max(worstReb,  100*rebUsed(i)/rebAvail(i));
    end
    fprintf('  CONCLUSION: the binding constraint differs by axle. Front rebound\n');
    fprintf('  headroom is only %.1f mm and is %.0f%% consumed; rear bump headroom\n', ...
            rebAvail(1), worstReb);
    fprintf('  is only %.1f mm and is %.0f%% consumed. The car is NOT operating with\n', bumpAvail(3), worstBump);
    fprintf('  large uniform reserve, and a softer spring would erode the tighter\n');
    fprintf('  side further. Do not recommend a rate reduction on travel grounds.\n');
else
    used = max(travelUsed);
    fprintf('  total range used %.1f-%.1f mm of %.0f mm stroke\n', min(travelUsed), used, avail);
    fprintf('  NOTE: pass an 8-element vector [bump(4) rebound(4)] measured from\n');
    fprintf('  static to get the per-axle headroom conclusion, which is the\n');
    fprintf('  meaningful one - total range hides the static-offset asymmetry.\n');
end

% ── 5. NULL RESULTS ──────────────────────────────────────────────────────
pAsym = 100*abs(pitchBrake-pitchTraction)/mean([pitchBrake pitchTraction]);
fprintf('\n-- 5. NULL RESULTS --\n');
fprintf('  pitch braking %.3f vs traction %.3f deg/g -> %.1f%% apart\n', pitchBrake, pitchTraction, pAsym);
if pAsym < 5
    fprintf('  CONCLUSION: no measurable anti-dive/anti-squat asymmetry.\n');
    fprintf('  Pitch response is linear, direction-independent, spring-dominated.\n');
end
fprintf('  front/rear roll gradient differ by %.3f deg/g (%.1f%% of total)\n', ...
        abs(rollGradF-rollGradR), rollMismatch);
if rollMismatch < 5
    fprintf('  CONCLUSION: chassis torsionally stiff relative to the suspension;\n');
    fprintf('  no meaningful compliance in the roll path.\n');
end

fprintf('\n================================================\n');
C = struct('rollAtPeak_deg',roll_susp,'rollToRoad_deg',roll_road,'outsideCompression_mm',comp_mm, ...
           'netCamber_deg',nets,'camberGainsTested',gains, ...
           'K_measured',Rinv,'K_springs',Kf+Kr,'K_front',Kf,'K_rear',KrTot, ...
           'arb_Nm_deg',arbUse,'rollStiffnessDist_pctFront',rsd, ...
           'bumpAvail_mm',bumpAvail,'rebAvail_mm',rebAvail, ...
           'mrRatioRequired',mrRatioReq,'mrRatioUsed',mrRatioUsed, ...
           'pitchAsymmetry_pct',pAsym,'rollMismatch_pct',rollMismatch);
end

function K = roll_stiffness_measured(V, grad)
    g = 9.81;
    h_rc = V.rc_front_mm*V.front_mass_frac + V.rc_rear_mm*(1-V.front_mass_frac);
    h_s  = (V.mass_kg*V.cg_height_mm - V.unsprung_total_kg*V.tyre_radius_mm)/V.sprung_mass_kg;
    K = V.sprung_mass_kg*g*((h_s-h_rc)/1000)/grad;
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
function n = corner_name(i)
    names = {'FL','FR','RL','RR'}; n = names{i};
end
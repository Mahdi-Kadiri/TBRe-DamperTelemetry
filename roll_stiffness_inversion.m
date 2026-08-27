function R = roll_stiffness_inversion()
% ROLL_STIFFNESS_INVERSION  Convert the measured roll gradient into a measured
% total suspension roll stiffness, and compare against the design model built
% from springs and anti-roll bars through the motion ratios.
%
% METHOD
%   At quasi-steady state the roll moment is reacted by the suspension:
%       K_roll * phi = m_s * g * arm * Ay        [Ay in g]
%   so  K_roll = m_s * g * arm / (dphi/dAy)
%   with arm = (sprung CG height) - (roll centre height).
%
%   Sprung CG is HIGHER than whole-vehicle CG, because unsprung mass sits
%   low (at wheel-centre height). Getting this wrong understates the roll
%   arm by ~13 mm on this car, ~5% on the answer.
%
% WHY THE COMPARISON IS THE POINT
%   The measured value contains everything real: springs, ARB, bushing and
%   rod-end compliance, chassis torsion, and any motion ratio that differs
%   from design. The design value contains only springs and ARB. Measured
%   below design means stiffness is being lost, and the gap is the finding.
%   Measured above design means an input is wrong, not that the car is
%   stiffer than designed.
%
% INPUTS come from tbre25_vehicle_data.m - edit values there, not here.
%
% LIMITATIONS
%   - Damper-derived roll excludes tyre vertical deflection, so this is
%     suspension roll stiffness. Correct pairing for a springs+ARB figure.
%   - Static 2D roll centres; migration under roll not included. This is now
%     the dominant uncertainty and needs a kinematics sweep, not more data.
%   - Gradient fitted on 0.1-3 Hz band-passed data: transient/corner-scale
%     response, comparable to but not identical with a steady-state skidpad
%     gradient.
%   - Spring rate units inferred as lb/in (300 N/mm gives ~7x the measured
%     stiffness). Confirm with the team.

V = tbre25_vehicle_data();
g = 9.81;
ROLL_GRAD_DEG_PER_G = 0.712;   % measured, FSG25 endurance, r = 0.904
ROLL_GRAD_SE        = 0.008;   % PLACEHOLDER - replace with the regression
                               % standard error or a per-lap bootstrap

% ── MEASURED ─────────────────────────────────────────────────────────────
h_rc = V.rc_front_mm*V.front_mass_frac + V.rc_rear_mm*(1-V.front_mass_frac);
ms   = V.sprung_mass_kg;
h_s  = (V.mass_kg*V.cg_height_mm - V.unsprung_total_kg*V.tyre_radius_mm)/ms;
arm  = (h_s - h_rc)/1000;
K_meas = ms*g*arm/ROLL_GRAD_DEG_PER_G;

fprintf('\n=== MEASURED ROLL STIFFNESS ===\n');
fprintf('  roll gradient   %.3f deg/g (r = 0.904, n = 34,481)\n', ROLL_GRAD_DEG_PER_G);
fprintf('  mass            %.0f kg total, %.0f kg sprung (unsprung %.0f kg: %.0f f / %.0f r per corner)\n', ...
        V.mass_kg, ms, V.unsprung_total_kg, V.unsprung_f_kg, V.unsprung_r_kg);
fprintf('  CG              %.1f mm whole vehicle -> %.1f mm sprung\n', V.cg_height_mm, h_s);
fprintf('  roll centre     %.1f mm weighted (front %.1f, rear %.1f)\n', h_rc, V.rc_front_mm, V.rc_rear_mm);
fprintf('  roll arm        %.1f mm\n', arm*1000);
fprintf('  K_measured      %.0f N.m/deg\n', K_meas);

% ── DESIGN: SPRINGS ──────────────────────────────────────────────────────
axleK = @(k, MR, t) k*MR^2*1000*t^2/2/57.2958;   % k N/mm, t m -> N.m/deg
mrSets = {'Pavel (favoured)', 0.90, 0.69; 'Oliver ("I think")', 0.928, 0.89};
fprintf('\n=== DESIGN, SPRINGS ONLY (%.2f N/mm both ends; no front ARB) ===\n', V.spring_rate_Nmm);
Kspring = NaN;
for i = 1:size(mrSets,1)
    Kf = axleK(V.spring_rate_Nmm, mrSets{i,2}, V.track_f_mm/1000);
    Kr = axleK(V.spring_rate_Nmm, mrSets{i,3}, V.track_r_mm/1000);
    fprintf('  %-20s MR %.3f/%.3f -> %5.0f + %5.0f = %6.0f N.m/deg  (measured %.0f, gap %+.0f)\n', ...
            mrSets{i,1}, mrSets{i,2}, mrSets{i,3}, Kf, Kr, Kf+Kr, K_meas, K_meas-(Kf+Kr));
    if i == 1, Kspring = Kf+Kr; end
end
fprintf(['  Oliver''s rear MR puts springs alone ABOVE the measured value before\n' ...
         '  any ARB is added - not physically possible. Evidence favours 0.69.\n']);

% ── ARB: PREDICTION OR COMPARISON ────────────────────────────────────────
gap = K_meas - Kspring;
if isnan(V.arb_rear_Nm_deg)
    fprintf('\n=== PREDICTION (rear ARB rate not yet read from the sheet) ===\n');
    fprintf('  springs alone %.0f, measured %.0f -> shortfall %.0f N.m/deg\n', Kspring, K_meas, gap);
    fprintf('  With no front ARB, the rear ARB is the ONLY other stiffness source.\n');
    fprintf('  >>> PREDICTED rear ARB contribution: %.0f N.m/deg at closest setting.\n', gap);
    fprintf(['  Check the TBRe25 ARB sheet against this. Note which convention it\n' ...
             '  uses - bar torsional stiffness, wheel rate, and roll stiffness\n' ...
             '  contribution differ by the ARB motion ratio squared.\n']);
else
    Kdes = Kspring + V.arb_rear_Nm_deg;
    err  = 100*(K_meas/Kdes - 1);
    fprintf('\n=== DESIGN vs MEASURED ===\n');
    fprintf('  springs %.0f + rear ARB %.0f = %.0f N.m/deg design\n', Kspring, V.arb_rear_Nm_deg, Kdes);
    fprintf('  measured %.0f -> %+.1f%%\n', K_meas, err);
    if abs(err) < 8
        fprintf(['  Inside the +/-4%%-per-10mm sensitivity on RC and sprung CG:\n' ...
                 '  the design model is CONSISTENT with the measured car. This data\n' ...
                 '  cannot resolve a discrepancy smaller than that band.\n']);
    elseif err < 0
        fprintf(['  Measured BELOW design - roll stiffness is being lost. Candidates\n' ...
                 '  in order: compliance in mounts/bearings/rod ends, ARB not acting\n' ...
                 '  as modelled, motion ratio lower in practice than assumed.\n']);
    else
        fprintf(['  Measured ABOVE design - suspect the inputs before the car.\n' ...
                 '  Most likely the roll arm (RC or CG height) or a motion ratio.\n']);
    end
end

% ── SENSITIVITY ──────────────────────────────────────────────────────────
fprintf('\n=== SENSITIVITY ===\n');
for d = [-10 10]
    a = (h_s - (h_rc+d))/1000;
    fprintf('  roll centre %+3d mm -> %6.0f N.m/deg (%+5.1f%%)\n', d, ms*g*a/ROLL_GRAD_DEG_PER_G, 100*(a/arm-1));
end
for d = [-10 10]
    a = (h_s + d - h_rc)/1000;
    fprintf('  sprung CG   %+3d mm -> %6.0f N.m/deg (%+5.1f%%)\n', d, ms*g*a/ROLL_GRAD_DEG_PER_G, 100*(a/arm-1));
end
for d = [-ROLL_GRAD_SE ROLL_GRAD_SE]
    fprintf('  gradient %+.3f    -> %6.0f N.m/deg (%+5.1f%%)\n', d, ms*g*arm/(ROLL_GRAD_DEG_PER_G+d), ...
            100*(ROLL_GRAD_DEG_PER_G/(ROLL_GRAD_DEG_PER_G+d)-1));
end
fprintf(['  Mass terms are now confirmed, so the dominant uncertainty is the\n' ...
         '  STATIC 2D roll centre. Refining it needs a kinematics roll sweep,\n' ...
         '  not more data from the team. The gradient itself contributes least.\n']);
fprintf('  NOTE: the gradient standard error is a placeholder - replace it.\n');

R = struct('K_measured',K_meas,'K_springs',Kspring,'arb_predicted',gap, ...
           'arm_mm',arm*1000,'sprungMass_kg',ms,'sprungCG_mm',h_s,'rc_mm',h_rc);
end

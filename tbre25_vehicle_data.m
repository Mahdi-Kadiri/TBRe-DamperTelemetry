function V = tbre25_vehicle_data()
% TBRE25_VEHICLE_DATA  Parameter set for the analysis in this repository.
%
% PUBLIC VERSION. This is the working parameter file with team-confidential
% content removed. What is here is exactly what the README already reports,
% so the tools reproduce every published number. What has been removed does
% not affect any result in this repository:
%
%   - Suspension hardpoint coordinates (team geometry IP). Used only to
%     derive the roll centre heights, which are given directly below.
%   - Structural member specifications.
%   - Aerodynamic coefficients.
%   - Four-post rig campaign data and the internal document adjudication
%     record.
%
% If you are adapting this for another car, replace every value below. The
% tools read from here and nowhere else, so a parameter is defined once.
%
% CONVENTIONS: mm and degrees throughout unless stated. Motion ratio is
% damper travel per unit wheel travel.

% ── MASS & DIMENSIONS ────────────────────────────────────────────────────
V.peak_lat_g      = 2.08;      % highest lateral acceleration in the run
V.mass_kg         = 305;       % total, driver included
V.front_mass_frac = 0.48;
V.wheelbase_mm    = 1530;
V.track_f_mm      = 1200;
V.track_r_mm      = 1190;
V.cg_height_mm    = 286.0;     % whole vehicle
V.tyre_radius_mm  = 203;       % unloaded

% ── SPRUNG / UNSPRUNG SPLIT ─────────────────────────────────────────────
% Sprung CG is HIGHER than whole-vehicle CG: unsprung mass sits low, at
% wheel-centre height. h_s = (m*h_cg - m_u*h_u)/m_s.
V.unsprung_f_kg     = 8;                     % per corner
V.unsprung_r_kg     = 13;                    % per corner
V.unsprung_total_kg = 2*8 + 2*13;            % 42 kg
V.sprung_mass_kg    = 305 - (2*8 + 2*13);    % 263 kg
V.sprung_cg_mm      = 299.3;

% ── MOTION RATIOS (damper / wheel) ──────────────────────────────────────
% Verified against the telemetry rather than assumed: on a torsionally
% stiff chassis front and rear roll angles must agree, and the rear dampers
% move 77.8% as much as the fronts in roll. That fixes the rear/front
% motion-ratio ratio at 0.784; the pair below satisfies the front-rear roll
% agreement to 2.2%. Motion ratio is travel-dependent in reality; this is
% the effective value at the operating point.
V.mr_front = 0.90;
V.mr_rear  = 0.69;

% ── ROLL CENTRES (front-view 2D projection, static) ─────────────────────
% Derived from the suspension hardpoints (withheld). Static values: roll
% centre migration under roll is not included and needs a kinematics sweep.
% This is the largest remaining sensitivity in the roll stiffness result,
% at roughly 4% per 10 mm.
V.rc_front_mm = 48.6;          % above ground
V.rc_rear_mm  = 53.3;

% ── SPRINGS & ANTI-ROLL BARS ────────────────────────────────────────────
V.spring_rate_lbin = 300;                    % front and rear, same
V.spring_rate_Nmm  = 300 * 0.175127;         % = 52.54 N/mm
V.arb_front        = 0;                      % no front ARB fitted
V.arb_rear_Nm_deg  = NaN;                    % roll stiffness contribution at
                                             % the chassis. Left as NaN so the
                                             % tools report the residual
                                             % attributable to it instead of
                                             % assuming a value.

% ── SETUP AS RUN ────────────────────────────────────────────────────────
V.setup.camber_static_deg   = -1.0;          % as run, per the VD engineer
                                             % present at the event
V.setup.toe_front_deg       = +1.0;          % toe OUT
V.setup.toe_rear_deg        =  0.0;
V.setup.caster_deg          = 3.18;
V.setup.damper              = 'VC03';
V.setup.damper_clicks_f     = [4 7];         % [bump rebound]
V.setup.damper_clicks_r     = [11 11];
% No force-velocity dyno curve exists; click settings only. This blocks
% pushrod load magnitudes and is recorded as a gap rather than estimated.

% ── TRAVEL ──────────────────────────────────────────────────────────────
% 55 mm is DAMPER/SPRING stroke, full bump to full rebound, confirmed from
% three independent sources. The position sensors read damper millimetres,
% so travel utilisation is computed with no motion ratio in the chain.
% Travel is NOT symmetric about static - static compression consumes a
% different fraction of stroke at each axle, so bump and rebound headroom
% must be reported separately.
V.setup.travel_available_mm     = 55;
V.setup.static_compression_f_mm = 19.6;      % from full rebound
V.setup.static_compression_r_mm = 30.5;

% ── KINEMATIC GAINS ─────────────────────────────────────────────────────
V.setup.camber_gain_deg_mm = -0.066;         % kinematics solver. Design
                                             % target range -0.05 to -0.10;
                                             % conclusions are reported
                                             % across that whole range.
V.tyre_peak_camber_deg     = [-1.5 -2.5];    % tyre working range

% ── TYRE VERTICAL RATE (four-post rig, measured at 1.0 bar) ─────────────
% Lets the tyre contribution to roll be computed rather than caveated.
% Tyre roll at 1 g, from axle load transfer over tyre rate x track^2/2:
% front 0.183, rear 0.126, mean 0.154 deg/g. The design submission quotes
% 0.171/0.174 independently - agrees to about 10%.
V.tyre_rate_f_Nmm = 186;       % per corner
V.tyre_rate_r_Nmm = 187;
V.tyre_roll_deg_g = 0.154;

% ── DAMPER POSITION SENSOR CALIBRATION ──────────────────────────────────
% mm = a*counts + b. Derived from a dual-stream fit on the raw and
% calibrated logger streams, lag-corrected (88 ms) with sample-held points
% excluded. Four sensors agree on scale to 0.12%; residual 0.92-1.19 mm.
% Order: FL FR RL RR.
V.cal_a = [-0.022606 -0.022611 -0.022585 -0.022604];
V.cal_b = [ 74.747    74.768    74.705    74.745  ];

% ── IMU AXIS MAP ────────────────────────────────────────────────────────
% The IMU is mounted rotated about the vertical axis, so the logger's
% channel names do not correspond to the physical axes. The tools identify
% axes empirically from kinematics at runtime rather than trusting these,
% and refuse to proceed on an unverified reference - the map below is for
% reference only.
V.imu.lateral_channel      = 'InlineAcc';    % r = +0.974 vs v*yawrate
V.imu.longitudinal_channel = 'LateralAcc';   % r = -0.870 vs dV/dt, SIGN-INVERTED
V.imu.rollrate_channel     = 'PitchRate';    % |r| = 0.75 vs damper-derived roll rate
V.imu.pitchrate_channel    = 'RollRate';
% Yaw rate is unaffected, as a rotation about the vertical axis predicts.

% ── DESIGN-MODEL VALUES (for cross-checking the measurement) ────────────
V.design.roll_grad_deg_g = 0.724;   % vs 0.712 measured: 1.7% agreement

% ── DATA QUALITY LIMITS OF THE SOURCE FILE ──────────────────────────────
% - Export rate exactly 20.00 Hz -> 10 Hz Nyquist.
% - The calibrated damper stream sample-holds ~20% of moving samples with
%   stalls up to 5.85 s. All analysis uses the raw ADC stream.
% - Wheel hop measured at 20-27 Hz on the rig is entirely unrecoverable
%   from a 20 Hz log. Adequate capture needs >= 60 Hz.
end

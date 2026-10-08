# Atmosphere — Surface-to-Orbit Analysis

A local, English-language launch-trajectory workspace with independently configurable liquid/solid stages, global launch-site selection, atmosphere profiles, constrained orbit targeting, and inspectable results.

## Online interface, local calculation

The GitHub repository can be imported into Vercel using the repository root and
the included `vercel.json`. Vercel builds only the React interface. The Python
solver runs on your computer; no optimizer or analysis files are deployed.

On the calculation computer, run `start-web.cmd https://YOUR-SITE.vercel.app`
with the exact production address. This authorizes that single website origin
and starts the engine on loopback. In the website, select **Local engine**, enter
`http://127.0.0.1:8000`, and click **Connect**. Allow local network access if your
browser requests it. The browser talks directly to your engine; Vercel does not
proxy the computation. No public tunnel is required. Different computers cannot
access this loopback engine.

Without an engine, the site opens in **Review mode**. Use **Open analysis** to
load a complete **Analysis JSON** exported from the local app. Results, events,
orbit plots and charts are displayed without recomputation; CSV export also
works offline. The most recently viewed result is retained in this browser's
IndexedDB, including when a previously connected engine is unavailable. Storage
is specific to this browser and website origin; use exported files to transfer
results between devices. Results are not publicly uploaded or cloud-synced.

Launch recommendations, terrain lookups, atmosphere profiles and new consistency
calculations require the local engine. Review mode does not fabricate these
outputs. New API result exports include the arrival Earth orientation so the
geographic globe remains available without the engine; older exports without
that orientation show a neutral globe while preserving orbital scale.

Local terrain, analysis runs, dependency caches, secrets, generated builds and
preview screenshots are excluded from Git. Historical validation reports under
`data/` are local artifacts and are not part of the published repository.

## Run on Windows

Double-click **`start.cmd`**, or run it from this folder. Keep its service window open. The app opens at **http://127.0.0.1:8000** and listens only on loopback.

The first setup requires Python **3.12+** (tested with 3.13), Node.js **22+**, and internet access to install dependencies and build the interface. Subsequent launches use the local build and virtual environment. `start.cmd -SetupOnly` prepares dependencies without launching; `start.cmd -NoBrowser` leaves browser opening to you.

In development, run `.venv\Scripts\python.exe -m uvicorn atmosphere.main:app --app-dir backend --host 127.0.0.1 --port 8000` and, in `frontend`, `npm run dev` or `pnpm dev`. The Vite development URL is http://127.0.0.1:5173. After frontend edits, run `npm run build` or `pnpm build` to update the production interface.

## Workflow

1. **Target Orbit:** start with at least one mission parameter. Perigee, apogee, inclination and advanced angles are independently optional. Empty fields remain null and never become optimization constraints. The final solution must still be bound, with perigee at or above 100 km. Presets explicitly fill their displayed parameters.
2. **Launch Site:** compare orbit-driven launch recommendations, then choose a marked pad or the corresponding list. Direct geometry matches, maneuver-required options and unpublished range envelopes are distinguished. Ranking estimates rotation benefit and an impulsive plane change at a reference perigee speed; it is not a fuel or full launch delta-v prediction. Vehicle capability is evaluated later. RAAN plus inclination enables estimated launch-plane crossing times over the next 24 hours. Select **Manual launch site** for arbitrary coordinates. Known or manual departure sectors can be edited as start/end bearing pairs; north-crossing sectors are supported. Coordinates and terrain elevation appear below the selector.
3. **Environment:** select NRLMSIS 2.1 or U.S. Standard Atmosphere 1976, then preview density/temperature. Automatic solar indices need data for the chosen date. When unavailable, explicitly select nominal conditions or supply F10.7, its 81-day average, and Ap. Nominal conditions are F10.7 = 150 sfu and Ap = 4; they are not weather observations.
4. **Vehicle & Stages:** define one payload and up to two serial stages. Browse booster or upper-stage engine profiles in the right column, configure any unpublished model inputs, then apply an engine to each stage. Set dry/propellant masses and engine count; advanced settings contain drag, throttle and restart inputs. Total initial mass is derived. Historical solid-stage designs remain readable.
5. **Flight Constraints:** configure atmospheric and optional space thrust–velocity cones for each stage, initial vertical guidance, optional Max-Q/load limits, and maximum mission time. The initial vertical command obeys the same cone; only speeds below 1 m/s use the explicit local-vertical definition.
6. **Analysis & Results:** choose propellant or time objective and complete-mission direct collocation (default) or legacy comparison, configure control/mesh resolution and structure budget under **Solver settings**, run/cancel, inspect supplied-target errors and all achieved orbital parameters, and open **Charts**. Download complete analysis JSON or time-series CSV. Input edits mark existing results **Outdated**.

Use the top-right controls to save/load scenario JSON. Inputs are also saved in this browser's local storage; analyses are persisted under `data/analyses`. Files in `examples/` provide ready-to-load studies. Example vehicle data is synthetic, not a validated real launcher.

The **Consistency check** in Vehicle & Stages and Analysis screens evaluates stage-by-stage ideal delta-v, carried upper-stage/payload mass and estimated initial pad thrust-to-weight. An optimistic central-field energy screen chooses the least energetic orbit permitted by the supplied apses and the implicit 100 km perigee floor. Its derivation uses `w = sqrt(2(E + mu/r_min))`, so `dw/dt <= |a_thrust|` when central gravity is conservative, no other force adds energy, and the trajectory stays outside `r_min`. The global radius is conservatively WGS84 polar radius minus 650 m. This simplified necessary-condition check ignores J2, atmospheric energy exchange and trajectory losses; passing is never a flight feasibility certificate. Vacuum Isp/full fuel/empty-stage disposal are optimistic assumptions. T/W uses standard-atmosphere pad pressure and local gravity, with 1.2 only a screening reference, not a flight constraint. The optional ideal payload ceiling is not a deliverable payload rating. Warnings remain advisory so an explicitly requested diagnostic analysis can still run.

**Flight diagnosis** uses each result's saved inputs and separates capacity warnings, exhausted optimization budget, descending radial velocity, Earth-intersecting osculating orbit and fuel still aboard. It does not relabel a cutoff trajectory as Impact without simulating collision. New analysis JSON includes the consistency report. Older saved results are diagnosed without rewriting them. Endpoints: `POST /api/consistency` and `GET /api/analyses/{id}/diagnostics`.

## Physical conventions

- Internal inputs, states and exports use SI. The UI also displays tonnes, kN, km and kPa with explicit labels.
- WGS84 ellipsoid; Astropy ITRS/GCRS orientation; central Earth gravity plus J2. Bundled IERS data avoids time-dependent automatic downloads inside the solver; extrapolation is disclosed in results.
- Orbit altitudes use the constant WGS84 equatorial radius, 6,378.137 km. Results describe the final osculating orbit, not a long-term averaged orbit.
- The 3D globe uses a WGS84 surface, an offline Natural Earth land mask/coastlines/country boundaries and the same kilometre scale as orbit traces. Astropy rotates geography into GCRS at launch for previews and at arrival for saved results. The static arrival globe is not a rotating animation of the historical flight. Map geometry is a display layer, not additional terrain or flight constraints.
- Earth rotation contributes to initial inertial velocity. Drag and steering use atmosphere-relative velocity; in vacuum this becomes velocity relative to rotating Earth.
- Vacuum mass flow is `vacuum thrust / (g0 × vacuum Isp)`. Ambient pressure adjusts Isp and thrust together. A solid curve contains normalized time (0..1) and thrust multiplier; its integrated impulse determines the full burn duration, preserving the specified propellant mass.
- An adaptive DOP853 integrator resolves burn/coast/fuel/separation/impact events. A continuous forward-shooting initial fit feeds normalized multiple shooting with explicit phase continuity and constrained SLSQP. Legal ignition-count templates are searched within the configured budget.
- The solver performs a feasibility search before objective improvement. Final candidates are replayed with stricter integration and denser sampling. A local solver's convergence flag is never accepted as proof of orbit insertion. A budget-limited result need not be a globally optimal trajectory.
- Independent phase-boundary states have explicit collision-surface constraints. Nonfinite, deeply underground and out-of-domain trial states are rejected before atmosphere queries. Invalid candidates produce finite failure residuals, cannot replace the retained best valid candidate, and do not abort the entire alternative search. Cancellation and genuine missing-environment errors remain explicit.
- Surface-crossing RK internal stages use atmosphere at the collision surface solely to bracket the impact event safely. This extension does not accept underground trajectories or remove valid below-sea-level launch sites. Direct atmosphere queries below -1,000 m are rejected.
- In legacy comparison mode, the initial continuous fit receives at most 40% of the total time budget; remaining time is shared across the remaining constrained searches, including their setup and integration work. Total time exhaustion, individual search time allowances, iteration limits, convergence and numerical stops are separately recorded in `optimizer.searches`. Strict verification runs outside the search budget and can exclude an individual failed candidate while retaining other valid flights. Continuously propagated baseline/initial candidates are retained for verification.
- Results show **Optimization & acceptance** separately: target tolerance failures explain flight acceptance; search termination explains why optimization stopped. Failed targets also distinguish per-search iteration limits from total time exhaustion, and explain when a discretized candidate misses during continuous flight replay. Older saved results retain their original data and are labelled as predating detailed termination reporting.
- Propellant minimization counts consumed **and discarded** propellant. Results separately report burned fuel, carried reserve, and fuel jettisoned with a stage.

## Complete-mission optimization

The default backend is **multi-phase Hermite–Simpson direct collocation + CasADi/IPOPT**. A bounded portfolio enumerates legal ignition-count combinations across all stages, including distributed restarts. Atmospheric ascent, coast, restart, orbit raising, out-of-plane steering and arrival coast share the same 3D dynamics. Position/velocity are linked at every boundary; separation discards the old stage and reserve and loads the next stage's fuel. Solid profiles remain fixed. Every stage remains part of the prescribed serial vehicle: payload maneuvering requires an explicitly modeled propulsion stage.

Only supplied final orbit parameters and optional earliest/latest arrival times are constrained. A parking-orbit or transfer construction initializes high-orbit studies; it never imposes a hidden intermediate orbit. The ascent fit receives up to 22% of the search budget. Coarse structure searches share the next allocation; refinement spends the remainder on the best continuously replayed structure. Each refinement level doubles state/control resolution. This is a bounded portfolio with a local NLP, not an exhaustive search or proof of global optimality/infeasibility.

CasADi differentiates the production flight equations, orbital-invariant objective and bound-orbit safeguards automatically. The existing Astropy quaternion interpolation and NRLMSIS/US76 atmosphere cells provide **exact piecewise interpolation slopes** through small callbacks, avoiding finite differences of the entire 13-input force node. The latitude/longitude/time dependence, below-sea-level extension, J2, Earth rotation, steering limits and solid profiles remain in the model. Terminal angular-element callbacks retain guarded finite differences; a custom atmosphere without slopes falls back to the original guarded node. No altitude-only atmosphere surrogate is used. IPOPT uses a limited-memory Hessian and a 0.001 initial barrier parameter for normalized variables. Nonsingular semilatus rectum/eccentricity-vector residuals avoid undefined apogee/argument behavior in trial trajectories. A mesh-feasible candidate must also pass a continuous flight replay before cost polishing; independent strict flight acceptance remains the final authority.

New plans explicitly use a local orbital steering frame, blending from launch-heading guidance over 5–15° away from vertical. This avoids the heading-projection singularity in orbital plane changes. Saved plans without a frame field retain the historical launch-heading convention, so successful reference flights remain reproducible. Space angle limits are explicit vehicle inputs, blending over 100–120 km altitude; blank inherits the atmospheric limit. These are thrust-to-velocity limits, not motor gimbal angles. No rigid-body attitude model is implied.

Progress separates the current NLP target error (in tolerance units), normalized constraint violation, and best continuously propagated flight error. A continuous candidate is not described as strictly verified. Returned candidates are independently replayed with DOP853, including integration nodes and control/solid-profile breakpoints in constraint checks. The result records selected structure, phase plan, resolutions, termination reasons, derivative method and CasADi version. Pending jobs also retain `scenario.json` before calculation.

The independent maneuver regression solves a finite burn from a 400 km circular orbit to a 400 × 600 km orbit inclined 3°, then verifies it with the production forward integrator. This demonstrates combined energy/plane steering; it is separate from surface-to-GEO feasibility. Existing surface-to-LEO reference flights are retained. See `tests/test_mission.py` for structure, fuel, arrival-window, steering-singularity, cancellation and transcription checks.

`tests/test_derivatives.py` compares accelerated node values to the independent NumPy force implementation, compares atmosphere/node slopes to central differences, and checks solid profiles, polar/equatorial pads and vacuum behavior. Strict DOP853 replay always uses the independent NumPy equations. Mission seed preparation stops when a continuous seed meets its temporary/final target, retaining that flight and transferring unused time to the NLP; deadline-returned seeds are also propagated/scored before mesh selection. This stopping rule does not change final target acceptance. Repeatable derivative timing (without profiling overhead) is available with `data/diagnostics/profile_collocation.py --finite-difference --label finite-difference` and `--label final-kernel`. Whole-mission offline benchmarks use `data/diagnostics/verify_mission.py`; budgets, iteration/structure caps and output labels are explicit and never alter browser inputs or the source result. Measurements, final-source LEO/GTO/GEO outcomes and remaining convergence limitations are recorded in [the performance validation report](data/diagnostics/performance-validation.md).

IPOPT's bound/slack initialization pushes are 1e-8 in normalized units: the default 0.01 would shift an initially zero coast by 10 s and distort a full-throttle/fuel seed. The feasibility phase stops once a connected NLP candidate meets the 0.8-tolerance safety margin; forward replay and final strict acceptance still follow. A 2,048-RHS evaluation window that advances less than 1 ms is rejected as stalled integration, covering chattering at the existing 1 m/s vertical-guidance switch without silently changing that steering rule.

## Mission Budget

Analysis & Results and the vehicle overview include a fast advisory Mission Budget.
Ideal reference, nominal and margin tiers use a two-body impulsive departure and
an aligned-apogee/node combined arrival maneuver. The ideal reference is loss-free,
not a certified global minimum. Unspecified apses use illustrative low-energy
values; advanced angles and arrival windows are explicitly marked unpriced.

Planning assumptions are saved in the scenario's optional `budget` field. Defaults
are gravity loss 1,800 m/s, drag 150 m/s, steering 150 m/s and a 10% contingency
delta-v allowance. They are editable estimates, not measured losses or confidence
bounds. The contingency is held as fuel in the final stage after arrival.

Inverse vacuum rocket sizing works backward from payload through fixed dry masses,
carrying the required upper-stage fuel in lower-stage mass ratios. Ascent delta-v
is allocated proportionally to the prescribed vehicle's vacuum stage contributions.
The table reports stage load shortages even when total available fuel appears
sufficient. This allocation is a reference, not optimized fuel distribution.
Solid loads cannot be resized by the estimate. Ignition, advanced-target and sized
pad T/W limitations are reported; actual steering and other flight limits require
the continuous solver. Estimates never modify vehicle inputs or trajectory controls.

Payload sensitivity uses four payloads with unchanged dry masses and assumptions.
Saved flights with apogee near GEO also show remaining ideal delta-v and an onward
GEO maneuver estimate. The combined onward maneuver assumes a plane intersection
at apogee, and does not certify the actual geometry or vehicle steering capability.
Budget edits do not mark a verified flight outdated; physical-input edits do.
Old scenario/result JSON remains readable, with explicit default budget assumptions.

## Physical model limits

This is a **3-DOF engineering preliminary analysis**, not a flight-certified guidance system. Rigid-body attitude, aerodynamic lift, structural/thermal dynamics, fairing events, recovery, third-body gravity and long-term orbit stability are outside this version. Atmospheric temperature is not skin temperature. High/long-duration orbits are modeled with Earth central gravity/J2 only.

Launch-site screening is preliminary. The bundled numeric envelopes cover Kennedy/Cape Canaveral (NASA Ames 2013 reference, 35–120°), Wallops (NASA 2013 handbook, 90–160°) and a conservative historical Vandenberg corridor (1972 reference, 172–200°). These dated references are not current operational permission. Vehicle-specific doglegs and mission approval can provide additional access, including polar flights from the Eastern Range. Other sites retain unknown azimuth envelopes rather than invented limits. Reference links are preserved in the shared catalog and shown alongside recommendations. The solver selects a continuous azimuth branch inside supplied sectors; strict replay checks the launch heading and first-stage ground-track bearing after 1 km downrange. This sector model does not resolve population polygons, drop zones, flight-termination envelopes or current NOTAMs. User-defined sectors are included in saved scenarios and result metadata.

Collision detection uses resolved launch terrain within 50 km of the launch point and the WGS84 ellipsoid beyond that region. A globally resolved mountain-clearance corridor is not implemented. Launch-site DEM selection must not be interpreted as full-route terrain clearance.

NRLMSIS profiles use continuous interpolation across a 1° latitude/longitude and one-hour grid with vertical log-density PCHIP. Conditions are climatological, not forecasts; wind is zero. Its provider interface supports future wind/weather/profile integrations. Above 1,000 km density/pressure are zero and temperature/Mach are unavailable. For below-ellipsoid sea-level heights, MSIS is hydrostatically extended from its lowest level. US76 is analytic through 86 km; higher levels use an interpolated reference table and approximate gas composition.

## Data, availability and attribution

- Known launch-pad coordinates: a bundled selection of 24 worldwide sites from [Launch Library 2 / TheSpaceDevs](https://thespacedevs.com/llapi), retrieved 2026-10-04. Each entry represents the named pad, not the entire spaceport. The list is illustrative rather than exhaustive and does not assert current operational status. Elevation comes from the DEM, not this catalog.

- [NOAA ETOPO 2022](https://www.ncei.noaa.gov/metadata/geoportal/rest/metadata/item/gov.noaa.ngdc.mgg.dem%3Aetopo_2022/html), DOI [10.25921/fd45-gt74](https://doi.org/10.25921/fd45-gt74), 15 arc-second nearest-cell surface/geoid samples. EGM2008 elevation + geoid height = WGS84 ellipsoid height. Local single-band GeoTIFF/NetCDF surface and geoid rasters can be uploaded; the per-file upload limit is 200 MB.
- [Natural Earth](https://www.naturalearthdata.com/) 1:10m land polygons classify ocean versus land. Shoreline detail is finite; the surface override is intentional.
- [NRLMSIS 2.1](https://ccmc.gsfc.nasa.gov/models/NRLMSIS~2.1/) via [pymsis](https://swxtrec.github.io/pymsis/). Acknowledge NRL and University of Colorado SWx TREC in derived publications; automatic geomagnetic inputs use pymsis/CelesTrak's source data. **MSIS2 commercial use requires contacting NRL**, as described in pymsis's model licensing documentation. This project's selected use is personal/academic research.
- [U.S. Standard Atmosphere 1976](https://ntrs.nasa.gov/citations/19770009539).
- [OpenFreeMap](https://openfreemap.org/) / OpenStreetMap provide browser basemap tiles and attribution. Fonts may load from Google Fonts; system fonts remain available if offline.

First-time basemap, DEM/land-mask and automatic solar-index access needs internet. Cached DEM samples and manual/nominal atmosphere conditions can be used without fresh data downloads. Full offline global maps/terrain are not bundled. External data failure is shown explicitly; it does not silently substitute a different model or zero terrain.

## Tests

```powershell
.venv\Scripts\python.exe -m pytest -q
cd frontend
npm test
npm run build
```

Physics tests cover coordinates/rotation, geoid conventions, two-body energy/angular-momentum conservation, vacuum rocket performance, reference atmospheres, thrust cones, mass/separation/restart events, no-liftoff versus impact, circular/equatorial/retrograde elements, and continuity of multiple-shooting states. A two-body Hohmann GEO coast/circularization reference validates the transfer geometry; it is not a certified end-to-end GEO launcher benchmark. Frontend tests cover orbit geometry and antimeridian-safe ground tracks.

The saved `examples/leo-msis-reference.json` and `leo-standard-reference.json` retain input snapshots and controls from successful 200 km insertions. The test suite independently replays both with strict integration, checks target tolerances and flight limits, and verifies the propellant ledger. The MSIS run reached 199.747 km perigee, 199.966 km apogee and 28.5012° inclination under explicitly selected nominal solar conditions. These are synthetic vehicle benchmarks; the budget-limited search does not establish a globally optimal trajectory. Load `examples/leo.json` as a scenario; the reference files also contain solver outputs and are intended for regression replay.

Launch-site screening is available at `POST /api/launch-sites/recommendations`. API documentation is at **http://127.0.0.1:8000/docs**. Environment previews and terrain queries run separately from the analysis process. One analysis runs at a time. Cancellation is checked between integration steps; server shutdown stops its worker. Interrupted analyses are labeled rather than left permanently running.

## Engine library and vehicle design

New designs use one payload and one or two serial stages. Vehicle & Stages presents a role-filtered engine library with manufacturer renders or photographs, propulsion facts, and source links. The eight initial references are Merlin Sea Level / Vacuum, BE-4, BE-3U, Rutherford Sea Level / Vacuum, Vinci and Vulcain 2.1. Catalog facts and online image URLs live in `shared/engines.json`, reviewed on 2026-10-05. Images are static views, not interactive CAD models; missing or unavailable online images show a fallback. Image credits belong to the respective manufacturers.

Unpublished thrust and Isp values must be entered explicitly before applying an engine. Published booster sea-level thrust determines the sea-level Isp model value through the existing constant-mass-flow pressure-response approximation. For vacuum engines, the sea-level Isp is an explicit model approximation, not a sea-level operating certification. Restart and throttle presets are conservative mission settings where documented limits are unavailable; review them in advanced settings. The source-backed profile is not a complete engine operating map.

The Merlin Vacuum profile uses the 981 kN thrust reference and throttle range from SpaceX's May 2025 payload guide. Rutherford's first-stage 311 s Isp is displayed as a pressure-unspecified reference; it is not imposed as vacuum Isp. Catalog source links distinguish manufacturer facts from required simulation inputs.

Published minimum throttle references are enforced by both input validation and the calculation API. Engine profiles without a published throttle range retain explicitly editable mission assumptions.

Engine count multiplies stage thrust and mass flow, while Isp stays per engine. Dry mass and propellant are the initial design inputs, including engine, tank and supporting-system masses. When Stage sizing is enabled, the joint solver may change these masses within the specified bounds. Engine identity, count, catalog version and numerical simulation inputs are saved with the scenario/results. Catalog ratings are checked by the API, and new analyses/imports exceeding two stages are rejected. Historical results and consistency reports remain readable; restoring an older three-stage workspace retains every stage until the user chooses which one to remove.

Restored legacy numerical inputs can be loaded and inspected, but a catalog engine must be selected for every stage before a new analysis can start.

## Joint stage mass optimization

Stage sizing in Vehicle & Stages enables dry mass and loaded propellant decision variables in the same complete-mission NLP as the flight controls. Payload, engine identities/counts, thrust, Isp and ignition capabilities remain fixed. Explicit mass bounds and `dry mass >= hardware + tank/structure ratio * propellant` prevent arbitrary mass reduction. Suggested bounds are planning assumptions, visible and editable; they are not manufacturer-certified tank or engine mass data. Solid motor mass/profile remain fixed. A minimum initial pad T/W is imposed using actual atmospheric pressure and rotating-Earth effective weight.

The loaded mass enters all phase dynamics, upper-stage carriage, separation fuel resets and fuel capacity constraints. Propellant cost includes burned and discarded propellant. Independent replay uses the optimized design, while `input_scenario` preserves the original input and `stage_sizing` records input/output masses, bounds and pad T/W. Historical snapshots still parse without enabling sizing. New API defaults enable it; fixed-mass comparison remains available with the toggle off.

At least the final 15% of the search allocation is reserved for bounded continuous shooting restoration using production DOP853 propagation. It can adjust masses, burn/coast durations, throttle and steering, correcting discretization error without changing acceptance tolerances. Cartesian cone corrections and nonsingular equatorial plane residuals allow plane steering at zero thrust angle; isolated zero-angle control endpoints are preserved so polar interpolation stays continuous. Once a continuous feasible solution exists, constrained SLSQP can improve the actual fuel/time cost. Numerical failures or time exhaustion retain the best feasible flight. Coast mesh intervals cluster at both endpoints to resolve rapid periapsis motion during long transfers. The final accepted design always passes separate strict replay. Local IPOPT/shooting searches do not establish global optimality or impossibility.

The continuous corrector uses rank-revealing bounded Newton prediction/correction before its joint least-squares fit. This permits coupled plane and energy correction when the first plane adjustment creates a second-order energy error. Retained flights still require all actual target/flight constraints. Coast intervals also satisfy `dt * max(speed/radius, sqrt(mu/radius^3)) <= 0.5`; this is a numerical resolution condition, not a flight limit. Mesh refinement enables longer/multiple-revolution phases. The search reports its coast resolution so an unresolved mesh cannot masquerade as a connected long-duration coast.

High-perigee initialization first fits an ascent into a low-perigee transfer with the requested apogee. Existing ascent commands are preserved when allocating legal restarts. The initial coast estimates remaining time to apogee from the actual terminal anomaly, then propagates production gravity to initialize a finite arrival burn. Arrival steering is parameterized against inertial velocity, avoiding a reversed prograde command when Earth-relative velocity points backward at high apogee. Every such direction is still projected into the physical air/Earth-relative thrust cone; the below-1 m/s vertical lock is unchanged. This seed does not constrain an intermediate orbit or prescribe the final maneuver.

After feasibility, small coupled mass/control SQP steps use smooth orbital and plane residuals before full SLSQP cost polishing. These steps limit departure from the verified trajectory while allowing actual dry/loaded-fuel changes. Only independently propagated, constraint-satisfying improvements are retained; the objective includes discarded fuel.

The plane feasibility residual respects an 80% margin inside the requested inclination tolerance instead of forcing an unnecessary exact equality. Equatorial cases use a vector residual outside that margin and a separate hemisphere guard; circular eccentricity errors scale with the requested orbit radius. Near a synchronous arrival, the seed can use an allowed small plane offset to avoid activating the explicit below-1 m/s Earth-relative vertical lock. Target values, tolerances and physical steering limits remain unchanged, and strict replay checks actual orbital elements. Progress distinguishes the current trial from the best continuously propagated error and explicitly shows when a verified feasible flight has been retained during cost optimization.

Run the isolated surface mission benchmarks with `.venv\Scripts\python.exe scripts\verify_stage_sizing.py --case all --budget 900 --label verification`. Each case saves original inputs, optimizer result, a fresh independent replay and a requirement audit under `data/stage-sizing`. A passing audit requires actual surface departure, changed masses, fixed payload/engines, target/flight constraints, mass bounds and balanced fuel. Orbital-only transfer tests are not counted as surface mission success.

Equatorial plane margins use the sine of the actual margin angle, so the vector residual and angular acceptance agree even at larger tolerances. Inertial arrival searches have a soft rotating-Earth speed conditioning merit below 3 m/s, avoiding unnecessary crossings of the unchanged 1 m/s vertical-lock discontinuity. This merit is not a flight constraint or part of the fuel/time objective. Final acceptance and retained cost improvements use the original physical targets and constraints. Each search reports its conditioning merit alongside coast resolution.

Final joint-sizing validation: **180 backend tests**, **15 frontend tests**, TypeScript and production build passed. Fresh complete surface optimization plus separate strict replay passed LEO, MEO, GTO and GEO with changed stage masses, fixed payload/engines and original target tolerances. The [full audit report](data/stage-sizing/angle-consistent/report.md) records masses, numerical settings and evidence. These local searches prove the returned designs satisfy the model; they do not prove a global minimum or that a fresh design improves on every archived feasible design.

## Varied system validation

The broader 12-case matrix covers different launch latitudes/sectors, polar and retrograde targets, partial orbit inputs, synthetic below-sea/elevated pads, manual MSIS conditions, different engines/payloads, fixed or jointly sized masses, and both optimization objectives. Run `scripts/verify_system_matrix.py --case all --budget 300 --workers 2 --label varied-v3` with the project Python environment. These are isolated command-line test runs, not an application parallel-analysis feature. A `STOP` file in the label directory cancels the test group. Fresh worker source hashes must match the source loaded by the worker; a changed implementation invalidates the audit.

The [current matrix report](data/system-validation/varied-v3/report.md) explicitly distinguishes unfinished or missed targets from passing independent strict replay. Generate it with `scripts/report_system_matrix.py --label varied-v3`. Original 1 km / 0.1° tolerances remain unchanged. Initial budgets are diagnostic; difficult cases may need a separately recorded longer search or a solver correction. No search miss is presented as physical impossibility.

System checks identified and fixed premature ignition events after a coast reaches the mission duration, incorrect final mass in an early impact without recorded rows, missing environment-data errors reported as numerical failures, invalid elevation coordinates reported as provider failures, and NumPy azimuth/boolean values preventing optimized results from being saved. Restoration commands are canonicalized before replay; verification flags use native booleans. Flight-duration termination is separately reported from the optimization time budget. New API worker tests use isolated job storage; `ATMOSPHERE_ANALYSES_DIR` can select a dedicated QA storage directory without touching user analysis files.

Frontend validation now rejects invalid tolerances, nonfinite/out-of-range targets, undefined angular elements and inconsistent arrival windows before analysis. Automated acceptance tests construct independent Kepler states for LEO/MEO/GTO/GEO, polar/retrograde and singular-element cases, and prove that Max-Q/load/arrival constraints reject an otherwise feasible orbit. The isolated browser check exercised scenario import, launch selection, engine profiles/two-stage limit, analysis/results/charts, CSV download and Outdated labeling.

The [latest combined evidence](data/system-validation/report.md) distinguishes fresh searches from saved candidates replayed with current code. Regenerate it with `scripts/report_system_validation.py`. Follow-up searches use `--budget 900` and retain the original tolerances. Ten successful saved flights have been strictly replayed with current code; the final two fresh 900-second GEO searches are recorded under `varied-final-v2`. The complete matrix now passes 12/12.

Continuous shooting now uses a bounded per-search segment cache. A hit requires identical design, initial/current state, time, commands, steering frame, atmosphere/frame identities and integration settings. Copies prevent separation or exported rows from mutating stored states; cancellation is checked before every hit. Changes to a vehicle or earlier flight segment trigger reintegration. Final independent verification does not use this cache. An isolated four-trial MEO benchmark returned identical strict results and measured 2.18 times faster last-burn duration trials (13.94 versus 6.39 seconds); this is not a claim about whole-optimizer speedup. Run `scripts/benchmark_restoration_cache.py` to reproduce the comparison.

Graph construction and NLP solve times are reported separately. Refinement is deferred when estimated graph construction would consume most of its time allocation; the remaining budget goes to correction of the continuously propagated flight. Deadlines during transfer-seed preparation retain candidates for strict verification; cancellation still propagates.

For high arrival orbits whose plane differs from the directly accessible departure plane, a capable three-ignition upper stage gets a parking/node/injection/arrival initialization. Production propagation positions transfer apogee in the destination plane. Finite arrival controls vary with changing inertial velocity, rather than holding a thrust-to-velocity offset that rotates the intended correction. The complete-mission NLP remains free to alter parking, coast, injection and arrival; no intermediate parking/node constraint or extra ignition capability is imposed. A separately corrected Cape-to-equatorial-GEO surface flight is retained in `examples/geo-cape-node-reference.json` for strict regression replay.

High-perigee missions reserve 30% of the search budget for continuous correction. When restart alternatives are available, the one-upper-burn alternative receives at most 6% of the total search budget. A failed first correction can restart around the retained physical candidate, refreshing its bounded control ranges while preserving all user design/flight bounds; strict acceptance controls whether another pass is needed. The correction merit includes mission-duration failure. With joint sizing enabled, the parking seed can use the upper-stage dry-mass floor allowed by the user's hardware/tank bounds, providing a useful arrival fuel reserve. Fixed-mass vehicles retain their supplied masses. The final two cold-start GEO searches are recorded under `data/system-validation/varied-final-v2`; the combined report identifies every search and replay separately.

Final varied-system verification: **228 backend tests**, **18 frontend tests**, TypeScript and production build passed. All **12 varied surface-flight audits** passed the original 1 km / 0.1 degree target tolerances and independent strict production replay. The combined report identifies ten saved-search replays and two fresh final searches explicitly. This verifies those returned designs, not a global minimum or universal convergence guarantee.


## Mission studio navigation

The English workspace fills the browser viewport, with a fixed navigation rail, command bar and section navigation. Forms switch between compact parameter pages rather than expanding vertically. Only catalogs, report lists, timelines and data tables scroll inside their own areas. Maps and charts resize to the available workspace. Narrow windows offer a pane selector for inputs versus visualization or stage settings versus engine selection, and the command bar includes a fullscreen control.

Orbit shape, orientation/timing and acceptance; launch location, launch time, departure sectors, recommendations, elevation and terrain files; atmosphere model and profiles; payload, independent stages, sizing and capacity; steering and limits; and optimizer/results each have dedicated sections. The engine library separates catalog, image/specification/source pages and application to a stage. Results separate overview, acceptance, diagnosis, trajectory, orbit comparison, fuel, charts, events and model metadata. Mission budget summaries, stage ledgers, loss allowances and maneuver estimates have separate pages. Draft sections remain navigable; input validation and strict local flight verification still control analysis acceptance.

With the calculation engine offline, the online workspace still supports mission editing, map/catalog selection, geometric launch ranking, scenario export and saved-result charts. Browser screening uses the same departure-plane and Earth-rotation equations as the local reference implementation; five reference cases cover unconstrained through retrograde planes. North/south mirror headings can tie. Precise RAAN windows, terrain lookup, atmosphere evaluation, fuel planning and optimization require the local engine. Save the mission and load it in the local workspace when the browser cannot reach loopback. Results are retained in this browser, not automatically synchronized between devices.

# Milestone 5.5 — physical vehicle integration

Active request: the attached Milestone 5.5 specification supersedes the IDE-selected M6 prompt.
Existing optimization artifacts are preserved; no new optimization work is part of this milestone.

## Baseline and gates

Read PROJECT_SPEC and M1–5 architecture/physics/replay documents before implementation. Run Release
CTest, M1–4 Python validations, replay-export validation, frontend unit/build/browser suites and retain
baseline performance. Inspect the supplied UI reference and both complete source glTF packages.

1. Assets: inspect metadata/dependencies; transform geometry into scene coordinates; find tire
   connected components (weld coincident vertices), validate four wheel clusters, infer axle axis
   and require an explicit documented forward sign/scale; classify material-specific components;
   export semantic body/steer/spin/caliper hierarchy and measured grounding manifest. Preserve
   original assets and texture/material distinctions. Gate: geometry/dependency/hierarchy tests.
2. Physics: retain default planar/quasi-static equations. Add explicit orthonormal road frames,
   gravity projection and optional sprung-body state in a separate coupled model. Reuse the exact
   production tire force law with suspension-supplied normal loads. Integrate body and vehicle
   together; export an optional versioned chassis companion. Gate: analytical grade, static loads,
   pitch/squat/roll, decay, energy and timestep convergence plus all legacy validation.
3. Presentation: load processed P1/F1 as visual choices independent of physics; road quaternion,
   road-constrained wheel contacts, dynamic body pose, replay-distance wheel spin, stationary
   calipers and stable second-order chase camera. Expose grounding/frame/chassis debug overlays.
   Gate: manifest/pose/steering/spin/switching tests and browser screenshots.
4. Environment: bounded elevated terrain, road material, explicitly configured curb sections,
   verges/runoff, continuous barrier geometry, deterministic instanced vegetation and lighting.
   Gate: mesh/frame/exclusion checks, visual review and measured desktop performance.
5. Documentation: reproducible commands, source credits, measured geometry, equations/estimates,
   complete test evidence, screenshots and honest limitations. Completion requires all gates.

## Proposed model and signs (derive before coding)

World/body physics uses x forward, y left, z up; gravity=(0,0,-9.80665). Road T is the
normalized (tx,ty,grade); L=(-ty,tx,0); N=T cross L. Bank is zero. Existing track s is
horizontal arc length; road speed projects onto horizontal kinematics using this road frame.

Sprung state q=(h,phi,theta) and qdot is displacement about preloaded static equilibrium.
phi is right-handed about forward x (left rises); theta is nose-up (rotation about minus y).
At contact i, body displacement d_i=h+y_i phi+x_i theta; compression c_i=-d_i.
F_i=F_i0+k_i c_i+c_d,i c_dot_i, selecting compression/rebound damping by velocity.
F_i0=m*g*(static axle fraction)/2 is the fixed level-road preload; nominal ride height bounds travel.
The initial proposal used grade-dependent preload; implementation fixes it so changed grade produces
the physical settling response rather than silently redefining equilibrium.

m*hdd=sum(F_i)-m*g_normal-D;
Ix*phidd=sum(y_i F_i)+height*sum(Fy_tire);
Iy*thetadd=sum(x_i F_i)+height*sum(Fx_tire).

Aero D acts at the CG and does not create an invented aero balance. Tire moments below CG
produce braking nose-down and outside roll without canned input-based animation. Dynamic F_i
feed the same nonlinear tires directly, with no additional quasi-static transfer. All mass is
lumped into the sprung body (no unsprung DOF); wheels remain constrained to the road. Negative
normal load or excessive deflection is an explicit envelope failure, never hidden clipping.
Road curvature inertial/heave excitation and full 3D angular momentum coupling remain beyond this
small-angle local-road model; document this approximation. Suspension values and inertias are
engineering estimates, not McLaren data.

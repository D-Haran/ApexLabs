# Fuel, energy and tires — approximate resource model

These states are integrated in the native dynamic contact model, at physical substeps
of at most 2 ms. They are absent from, and do not modify, the preserved earlier models.
Fixed-condition Phase B calibration disables resources explicitly; driving enables them.

Fuel adds mass at the sprung CG. The P1 reference mass includes an estimated 30 kg of
fuel; F1's 795 kg excludes fuel. No tank-induced CG migration is claimed. Delivered
positive tire work, divided by drivetrain and thermal efficiencies, determines engine
fuel flow, plus estimated idle consumption. No longitudinal wheel-spin state exists;
free-spinning engine losses are not resolved. F1 power is limited upstream by
`min(100, .009*rpm+5.5) kg/h`, consistent with the cited 2022 fuel-flow envelope.
Fuel exhaustion removes engine power. LHV and efficiencies are estimates.

Hybrid mechanical power is limited by available battery energy, motor efficiency and
remaining lap deployment budget. Battery discharge includes electrical losses. The
F1 configuration limits mechanical deployment to 120 kW; its electrical budget is
conservatively limited to 4 MJ/lap. Rear-wheel braking work can replenish the store,
after estimated recovery efficiency, subject to 120 kW, 2 MJ/lap and capacity limits.
Recovery replaces a portion of already requested braking work; it does not add extra
braking force. No MGU-H model or full brake-by-wire controller is claimed. Finish-line
crossings reset lap counters, never fuel or SOC. P1 capacity is an explicit 4.7 kWh
engineering assumption, not a verified OEM capacity; regenerative braking is disabled.

For each tire, surface and carcass obey two lumped heat balances. Surface input is
positive lateral slip work times a heat fraction plus half of rolling deformation
work. The other half heats the carcass. Surface/carcass conduction is equal and
opposite; air convection increases with speed; road conduction requires contact.
Longitudinal slip heating cannot be measured without wheel-spin dynamics and is
omitted. Tread and carcass heat capacities and all heat-transfer coefficients are
estimates in the vehicle JSON with provenance classes.

Wear integrates dissipated energy, multiplied by an overheating factor, and is
bounded to [0,1]. Temperature changes the reference friction multiplier through a
smooth Gaussian optimum with a 0.65 floor; wear reduces it by up to 30%. This
multiplier enters the existing nonlinear tire-force law. Pressure is not modeled.
These are generic thermal/wear surrogates, not proprietary Pirelli or McLaren maps.

The two-node structure and dependence of wear on thermal state and dissipated
contact power follow the *class* of models studied by
[West et al., Optimal tyre management for a high-performance race car](https://repository.up.ac.za/handle/2263/83745).
Coefficients here are not fitted to that paper's tire or to measured racing tires.
F1 limits: [McLaren MCL36 technical specification](https://www.mclaren.com/racing/formula-1/2022/car-launch/mclaren-mcl36-technical-specification/)
and [FIA 2022 technical regulations, issue 3](https://api.fia.com/sites/default/files/2022_formula_1_technical_regulations_-_iss_3_-_2021-02-19.pdf).

`apexlab.resources` tests fuel/electrical conservation, limits, exhaustion, recovery,
contact-dependent heating/wear, cooling, grip feedback and timestep refinement.

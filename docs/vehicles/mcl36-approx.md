# McLaren MCL36 / 2022 F1-inspired approximate model

`configs/vehicles/mcl36-dynamic.json` and `mcl36-chassis-approx.json` define physical differences from
the P1. Selecting an F1 visual mesh alone still does not select this physical model.

[McLaren's technical specification](https://www.mclaren.com/racing/formula-1/2022/car-launch/mclaren-mcl36-technical-specification/)
lists 795 kg including driver, excluding fuel; a front weight distribution range of 44.5–46%;
18-inch wheels; a 1.6 L V6; a 15,000 rpm limit; 100 kg/h fuel flow above 10,500 rpm; eight forward
gears; limited-slip differential; rear brake-by-wire; and 120 kW MGU-K with 4 MJ deployment / 2 MJ
recovery per lap. The 795 kg value is the requested McLaren specification, not a claim about every
later regulation amendment or race setup. Differential and brake-by-wire hardware are provenance
anchors; detailed mechanisms are not simulated.

All numerical geometry/inertia/tire/suspension fields in the chassis file are **estimated**, except
the **published** mass and the **derived** CG axle positions from a 3.6 m estimated wheelbase and
45% assumed front sprung weight. Corner stiffness, damping, contact properties and aerodynamic
coefficients are not McLaren or Pirelli data. ICE power and torque envelope, all ratios and shift
time are **estimated**; the rpm and hybrid-power limits are **published**. The dynamic configuration
tags each value individually. No fixed top speed is imposed.

The aero surrogate has front/rear wing area plus a floor contribution:

```
floor_ClA = configured_ClA * exp(-((mean_height-optimum)/width)^2)
            * sigmoid((mean_height - 0.015 m) / 0.007 m)
```

The two sigmoid heights are **estimated** numerical floor/stall properties. Mean height is queried
from the real road below front/rear body points. The floor's front share starts at 45% and migrates
by an estimated 2/m times rear-minus-front height, bounded between 30% and 60%. Wing and floor
forces are applied through their resultant center of pressure. Pitch and ride height therefore
alter both downforce and balance. This is not an identified MCL36 aero map or porpoising model.

DRS is a first-order physical aero state: opening reduces configured rear wing downforce and total
drag, with a resulting forward balance shift. Brake demand closes it. No speed is added directly.
The road/race preset affects only modeled aero area; all setup changes must act on real parameters.

The independent benchmark records acceleration, braking and top speed as **predictions**, without
calling them published MCL36 performance. Those values depend on the approximate aero/tires and
are not a circuit lap-time calibration.

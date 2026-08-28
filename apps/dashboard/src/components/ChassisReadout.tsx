import { useReplay } from "../state/replay";
export function ChassisReadout() {
  const { sample } = useReplay(),
    b = sample?.chassis;
  return (
    <div className="chassis-readout" data-testid="chassis-readout">
      {b ? (
        <>
          <b>SIMULATED CHASSIS</b>
          <span>Heave {(b.heave_m * 1000).toFixed(1)} mm</span>
          <span>Roll {((b.roll_rad * 180) / Math.PI).toFixed(2)}°</span>
          <span>Pitch {((b.pitch_rad * 180) / Math.PI).toFixed(2)}°</span>
          <span>
            Compression FL / FR / RL / RR:{" "}
            {[
              b.compression_fl_m,
              b.compression_fr_m,
              b.compression_rl_m,
              b.compression_rr_m,
            ]
              .map((v) => (v * 1000).toFixed(1))
              .join(" / ")}{" "}
            mm
          </span>
        </>
      ) : (
        <span>
          Chassis dynamics unavailable in this recorded planar session
        </span>
      )}
      <small>
        Green: wheel contacts · Blue: wheel centers · Amber: CG · Wheel
        rotation: kinematic approximation
      </small>
    </div>
  );
}

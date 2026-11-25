from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from amr_validation import metrics
from amr_validation.recorder import Recorder


# Free-running recorder that writes CSV files and prints a KPI summary on Ctrl+C
class KpiRecorder(Node):
    def __init__(self):
        # Simulation time keeps the recorder clock consistent with the message stamps
        super().__init__("kpi_recorder", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.out = Path(self.declare_parameter("output_dir", "/tmp/amr_kpi").value)
        self.name = self.declare_parameter("name", "run").value
        self.rec = Recorder(self)
        self.rec.start()
        self.get_logger().info("recording ground truth, AMCL and safety state ... Ctrl+C to finish")

    def summary(self) -> str:
        gt, est, saf = self.rec.arrays()
        err = metrics.pose_error(gt, est)
        lines = [f"samples: gt={len(gt)} amcl={len(est)} safety={len(saf)}",
                 f"distance driven: {metrics.path_length(gt[:, 1:3]):.2f} m",
                 f"localization RMSE xy: {err.rmse_xy:.3f} m (max {err.max_xy:.3f}), "
                 f"yaw RMSE: {err.rmse_yaw:.3f} rad"]
        if len(saf):
            lines.append(f"min clearance: {np.nanmin(saf[:, 2]):.3f} m, "
                         f"protective stops: {int(np.sum(np.diff((saf[:, 1] >= 2).astype(int)) == 1))}")
        return "\n".join(lines)


def main():
    rclpy.init()
    node = KpiRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rec.stop()
        node.rec.save(node.out, node.name)
        print(node.summary())
        print(f"CSV files written to {node.out}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

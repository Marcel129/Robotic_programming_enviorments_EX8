import rclpy
from rclpy.node import Node
import g2o
import numpy as np

class PoseGraphOptimizer(Node):
    def __init__(self):
        super().__init__('pose_graph_optimizer')
        self.optimizer = self.create_optimizer()
        self.add_vertices_and_edges()
        self.optimize()
        self.print_optimized_poses()

    def create_optimizer(self):
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(solver))
        return optimizer

    def add_vertices_and_edges(self):
        # Add robot pose vertices
        initial_poses = {
            0: (0, 0, 0),
            1: (2.1, 0, 0),
            2: (4.0, 0, 0)
        }
        for idx, pose in initial_poses.items():
            v = g2o.VertexSE2()
            v.set_id(idx)
            v.set_estimate(g2o.SE2(*pose))
            v.set_fixed(idx == 0)  # Fix the first pose
            self.optimizer.add_vertex(v)

        # Add landmark vertices
        landmarks = {
            3: (0.5, 1.0),
            4: (3.5, 1.0)
        }
        for idx, position in landmarks.items():
            v = g2o.VertexPointXY()
            v.set_id(idx)
            v.set_estimate(np.array(position))
            self.optimizer.add_vertex(v)

        # Define constraints (edges)
        constraints = [
            (0, 1, (2.1, 0, 0)),    # c1
            (1, 2, (1.9, 0, 0)),    # c2
            (0, 3, (0.5, 1.0)),     # c_{01}
            (1, 3, (-1.5, 1.0)),    # c_{11}
            (1, 4, (1.0, -1.0)),    # c_{12}
            (2, 4, (-1.0, -1.0))    # c_{22}
        ]

        information_matrix = np.identity(3)
        for (i, j, measurement) in constraints:
            if j < 3:  # Edge between poses
                edge = g2o.EdgeSE2()
                edge.set_vertex(0, self.optimizer.vertex(i))
                edge.set_vertex(1, self.optimizer.vertex(j))
                edge.set_measurement(g2o.SE2(*measurement))
                edge.set_information(information_matrix)
            else:  # Edge between pose and landmark
                edge = g2o.EdgeSE2PointXY()
                edge.set_vertex(0, self.optimizer.vertex(i))
                edge.set_vertex(1, self.optimizer.vertex(j))
                edge.set_measurement(np.array(measurement[:2]))
                edge.set_information(np.identity(2))
            self.optimizer.add_edge(edge)

    def optimize(self):
        self.optimizer.initialize_optimization()
        self.optimizer.optimize(10)

    def print_optimized_poses(self):
        for i in range(3):
            v = self.optimizer.vertex(i)
            est = v.estimate()
            self.get_logger().info(f'Pose {i}: x={est.translation()[0]:.2f}, y={est.translation()[1]:.2f}, theta={est.rotation().angle():.2f}')

        for i in range(3, 5):
            v = self.optimizer.vertex(i)
            est = v.estimate()
            self.get_logger().info(f'Landmark {i}: x={est[0]:.2f}, y={est[1]:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphOptimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

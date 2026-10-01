import math

import rclpy
from rclpy.node import Node
from sim_interfaces.srv import SolveTSP

XY = tuple[float, float]


def tsp_order(start: XY, pts: list[XY]) -> list[int]:
    """ visiting order of pts near start, using near-neigh. seed w 2opt"""
    n = len(pts)
    if n <= 1:
        return list(range(n))

    def d(a: XY, b: XY) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    remaining = set(range(n))
    order: list[int] = []
    cur = start
    while remaining:
        nxt = min(remaining, key=lambda i: d(cur, pts[i]))
        order.append(nxt)
        remaining.remove(nxt)
        cur = pts[nxt]

    path = [start] + [pts[i] for i in order]
    idx = [-1] + order
    improved = True
    while improved:
        improved = False
        for i in range(1, n):
            for j in range(i + 1, n + 1):
                a, b, c = path[i - 1], path[i], path[j]
                old = d(a, b)
                new = d(a, c)
                if j < n:
                    e = path[j + 1]
                    old += d(c, e)
                    new += d(b, e)
                if new < old - 1e-9:
                    path[i : j + 1] = path[i : j + 1][::-1]
                    idx[i : j + 1] = idx[i : j + 1][::-1]
                    improved = True
    return idx[1:]


class TravelingSalesman(Node):
    """service node for solving traveling salesman problem"""

    def __init__(self) -> None:
        super().__init__("traveling_salesman")
        self.declare_parameter("solve_service", "/solve_tsp")
        solve_service = str(self.get_parameter("solve_service").value)
        self.service = self.create_service(SolveTSP, solve_service, self._handle_solve_tsp)
        self.get_logger().info(f"Serving TSP ordering on {solve_service}")

    def _handle_solve_tsp(
        self, request: SolveTSP.Request, response: SolveTSP.Response
    ) -> SolveTSP.Response:
        if len(request.x) != len(request.y):
            self.get_logger().error("SolveTSP called with mismatched x/y lengths")
            response.success = False
            response.order = []
            return response

        pts = list(zip(request.x, request.y))
        try:
            order = tsp_order((request.start_x, request.start_y), pts)
        except Exception as exc:
            self.get_logger().error(f"TSP solve failed: {exc}")
            response.success = False
            response.order = []
            return response

        response.success = True
        response.order = [int(i) for i in order]
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TravelingSalesman()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

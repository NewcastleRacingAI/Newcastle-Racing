from nrai_perception.node import main as handle_perception
from nrai_pathplanning.node import main as handle_pathplanning
from nrai_controller.node import main as handle_controller
from src.nrai_carmaker.nrai_carmaker_server import main as handle_simulator
from multiprocessing import Process, Queue
import argparse
import logging
import subprocess
import os
from typing import Literal

class Args(argparse.Namespace):
    logger_format: str = "%(asctime)s {%(processName)s} [%(levelname)s] %(filename)s:%(lineno)d => %(message)s"
    verbosity: int = logging.INFO
    zed: bool = False
    camera_topic: str = "/nrai/camera"
    planning_topic: str = "/nrai/planning"
    control_topic: str = "/nrai/control"
    simulator_host: str = "127.0.0.1"
    simulator_bind: str = "0.0.0.0"
    simulator_port: int = 8000
    topics: dict[str, Queue] = {}
    simulator_exe: str = "./src/nrai_carmaker/sensor_client.exe"
    carmaker_host: str = "127.0.0.1"
    carmaker_ports: list[str] = [2210, 2211]
    mode: Literal["all", "nrai-only", "simulator-only"] = "all"


def parse(args: list[str] | None = None):
    default_args = Args()
    parser = argparse.ArgumentParser("nrai_perception")
    parser.add_argument("-z", "--zed", action="store_true", default=default_args.zed, help="Whether we are using the zed camera or a CarMaker simulation")
    parser.add_argument("-ct", "--camera-topic", type=str, default=default_args.camera_topic, help="Topic simulator => perception. Used for both RGB and Depth camera from CarMaker")
    parser.add_argument("-pt", "--planning-topic", type=str, default=default_args.planning_topic, help="Topic perception => planning")
    parser.add_argument("-crt", "--control-topic", type=str, default=default_args.control_topic, help="Topic planning => control")
    parser.add_argument("-sh", "--simulator-host", type=str, default=default_args.simulator_host, help="Ip of the machine the simulator node is running on")
    parser.add_argument("-sb", "--simulator-bind", type=str, default=default_args.simulator_bind, help="Ip used to bind the simulator node server")
    parser.add_argument("-sp", "--simulator-port", type=int, default=default_args.simulator_port, help="Port the simulator node is listening on")
    parser.add_argument("-v", "--verbosity", type=int, default=default_args.verbosity, choices=[10, 20, 30, 40, 50], help="Level of verbosity. 10 => Debug ... 50 => Critical")
    parser.add_argument("-se", "--simulator-exe", type=str, default=default_args.simulator_exe, help="Path to the exe file that captures simulated data from CarMaker and sends it to the simulator node")
    parser.add_argument("-ch", "--carmaker-host", type=str, default=default_args.carmaker_host, help="Ip of the CarMaker camera RSI")
    parser.add_argument("-cp", "--carmaker-ports", type=str, nargs="*", default=default_args.carmaker_ports, help="Ports corresponding to all the CarMaker sensors")
    parser.add_argument("-m", "--mode", type=str, choices=["all", "nrai-only", "simulator-only"], default=default_args.mode, help="Which processes to launch. nrai-only will only launch the nrai python processes, simulator-only will only launch the CarMaker => NRAI executables")
    return parser.parse_args(args, namespace=default_args)

def main(sys_args: list[str] | None = None):
    args = parse(sys_args)

    logging.basicConfig(format=args.logger_format, level=args.verbosity)

    args.topics = {args.camera_topic: Queue(), args.planning_topic: Queue(), args.control_topic: Queue()}

    # NRAI nodes
    nodes = (
        Process(name="nrai_controller", target=handle_controller, args=[args], daemon=True),
        Process(name="nrai_pathplanning", target=handle_pathplanning, args=[args], daemon=True),
        Process(name="nrai_perception", target=handle_perception, args=[args], daemon=True),
        Process(name="nrai_simulator", target=handle_simulator, args=[args], daemon=True),
    ) if args.mode != "simulator-only" else tuple()

    for node in nodes:
        node.start()

    # CarMaker simulated sensors
    carmaker_exes = []
    if args.mode != "nrai-only" and not args.zed:
        assert os.path.exists(args.simulator_exe)
        carmaker_exes = [subprocess.Popen([args.simulator_exe, '-p', str(port), '-d', args.simulator_host, "-x", str(args.simulator_port)]) for port in args.carmaker_ports]

    try:
        for node in nodes:
            node.join()
    except KeyboardInterrupt:
        logging.getLogger().info("All nodes are closed")

    for carmaker_exe in carmaker_exes:
        carmaker_exe.kill()

if __name__ == "__main__":
    main()

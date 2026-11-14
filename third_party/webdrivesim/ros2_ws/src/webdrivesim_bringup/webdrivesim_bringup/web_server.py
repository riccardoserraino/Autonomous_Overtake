#!/usr/bin/env python3
"""
webdrivesim_web_server — serves the WebDriveSim browser simulator and tells it which world to load.

The simulator is a web page (3D in the browser). This node:
  * serves the page on http://<host>:<port>/            (default 8080)
  * serves the world files on /worlds/<name>.json       (from worlds_dir)
  * serves /session.json = {world, control, sync, ...}  -> the open page polls it every 2 s
    and loads the world automatically, so you only need to open the page ONCE.
The page then talks ROS 2 through rosbridge (ws://<host>:9090), started by the launch file.

Change the world at runtime:
    ros2 param set /webdrivesim_web_server world motorway_overtake.json
"""
import json
import os
import threading
import time
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

try:
    from ament_index_python.packages import get_package_share_directory
    SHARE = get_package_share_directory('webdrivesim_bringup')
except Exception:  # running from source
    SHARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def default_worlds_dir():
    """worlds folder: $WEBDRIVESIM_WORLDS, then /ros2_ws/worlds (Docker), then the repo next to the workspace."""
    for d in (os.environ.get('WEBDRIVESIM_WORLDS', ''), '/ros2_ws/worlds',
              os.path.join(os.getcwd(), 'worlds'), os.path.join(os.getcwd(), '..', 'worlds')):
        if d and os.path.isdir(d):
            return os.path.abspath(d)
    return os.path.join(SHARE, 'worlds')


class Handler(SimpleHTTPRequestHandler):
    """Static files + /session.json + /worlds/ (directory listing as JSON)."""

    def __init__(self, *args, node=None, **kw):
        self.node = node
        super().__init__(*args, directory=node.web_dir, **kw)

    def log_message(self, fmt, *args):  # keep the terminal clean
        pass

    def send_json(self, obj, code=200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/session.json':
            return self.send_json(self.node.session())
        if path in ('/worlds', '/worlds/'):
            files = sorted(f for f in os.listdir(self.node.worlds_dir) if f.endswith('.json'))
            return self.send_json({'worlds_dir': self.node.worlds_dir, 'worlds': files})
        if path.startswith('/worlds/'):
            name = os.path.basename(path[len('/worlds/'):])
            from urllib.parse import unquote
            f = os.path.join(self.node.worlds_dir, unquote(name))
            if not os.path.isfile(f):
                return self.send_json({'error': f'world not found: {name}', 'worlds_dir': self.node.worlds_dir}, 404)
            data = open(f, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path == '/':
            self.path = '/index.html'
        return super().do_GET()


class WebServer(Node):
    def __init__(self):
        super().__init__('webdrivesim_web_server')
        self.declare_parameter('port', 8080)
        self.declare_parameter('web_dir', os.path.join(SHARE, 'web'))
        self.declare_parameter('worlds_dir', '')
        self.declare_parameter('world', 'empty_highway.json')
        self.declare_parameter('control', 'ros')          # ros | auto | manual
        self.declare_parameter('sync', False)             # CARLA-like synchronous mode
        self.declare_parameter('autostart', True)
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('open_browser', False)
        p = lambda n: self.get_parameter(n).value
        self.web_dir = p('web_dir')
        self.worlds_dir = p('worlds_dir') or default_worlds_dir()
        self.rev = int(time.time() * 1000)
        self.add_on_set_parameters_callback(self.on_params)
        port = int(p('port'))
        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), partial(Handler, node=self))
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        url = f'http://localhost:{port}/'
        world = p('world')
        ok = os.path.isfile(os.path.join(self.worlds_dir, world))
        self.get_logger().info(
            '\n' + '=' * 64 +
            f'\n  WebDriveSim is served on  {url}\n  Open it in your browser (Chrome / Firefox). Keep the tab open.\n'
            f'  world      : {world}{"" if ok else "   <-- NOT FOUND in worlds_dir"}\n'
            f'  worlds_dir : {self.worlds_dir}\n  control    : {p("control")}   sync: {p("sync")}\n'
            f'  rosbridge  : ws://localhost:{p("rosbridge_port")}\n' + '=' * 64)
        if p('open_browser'):
            threading.Timer(1.5, lambda: webbrowser.open(url)).start()

    def session(self):
        p = lambda n: self.get_parameter(n).value
        return {'rev': self.rev, 'world': p('world'), 'control': p('control'), 'sync': bool(p('sync')),
                'autostart': bool(p('autostart')), 'ros_url': f'ws://localhost:{p("rosbridge_port")}'}

    def on_params(self, params):
        for prm in params:
            if prm.name == 'worlds_dir' and prm.value:
                self.worlds_dir = prm.value
            if prm.name in ('world', 'control', 'sync', 'autostart'):
                self.get_logger().info(f'{prm.name} -> {prm.value} (the browser follows within 2 s)')
        self.rev = int(time.time() * 1000)
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = WebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

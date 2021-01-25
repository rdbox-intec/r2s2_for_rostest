#!/usr/bin/env python3

from swagger_server.controllers.queuing_manager import QueuingManager
import connexion
import threading
import os
from swagger_server.controllers.lifecycle_manager import LifecycleManager
from swagger_server import encoder
import swagger_server.config


def main():
    app = connexion.App(__name__, specification_dir='./swagger/')
    app.app.json_encoder = encoder.JSONEncoder
    app.add_api('swagger.yaml', arguments={'title': 'RDBOX Robotics Simulator System'})

    # Validation
    if "RRSS_INTERNAL_AUTH_USER" not in os.environ:
        print('(Warning) Set the default value as the authentication information for internal communication. Use environment variables to override the authentication information.')
    if "RRSS_INTERNAL_AUTH_PASS" not in os.environ:
        print('(Warning) Set the default value as the authentication information for internal communication. Use environment variables to override the authentication information.')

    # Multi Thread
    ctx_lm = {"lock": threading.Lock()}
    th1 = LifecycleManager(ctx_lm)
    th1.start()
    ctx_qm = {"lock": threading.Lock()}
    th2 = QueuingManager(ctx_qm)
    th2.start()

    # Loop
    app.run(port=swagger_server.config.get('network', 'port'))


if __name__ == '__main__':
    main()

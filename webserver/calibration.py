# from urllib import request
from flask_restful import Resource
import sys
import subprocess
import pexpect
from flask import Flask, stream_with_context, Response, render_template, request

p = None


class CalibrateGyroOutput(Resource):

    def get(self):

        def generate():
            global p
            while not p.eof():
                strLine = p.readline().decode("utf-8")
                yield strLine

            # for row in iter_all_rows():
            #     yield ','.join(row) + '\n'

        return Response(generate(), mimetype='text/csv')

    def post(self):
        if p is None:
            raise RuntimeError("Error! process not started yet")
        else:
            p.sendline('')


class Calibration(Resource):

    def get(self):

        print(request.args['calibration_type'])
        if request.args['calibration_type'] == 'gyroscope':
            command = "rc_calibrate_gyro"
        elif request.args['calibration_type'] == 'accelerometer':
            command = "rc_calibrate_accel"
        elif request.args['calibration_type'] == 'magnetometer':
            command = "rc_calibrate_mag"
        elif request.args['calibration_type'] == 'ESCs':
            command = "/home/debian/bin/calibrate_escs.sh"
        elif request.args['calibration_type'] == 'DSM':
            command = "rc_calibrate_dsm"
        else:
            raise RuntimeError("Error! unsupported command given")
        # command = "./test.sh"
        global p
        print(f'running command {command}')
        p = pexpect.spawn(command)


# if __name__ == '__main__':
#     obj = CalibrateGyro()

#     obj.get()

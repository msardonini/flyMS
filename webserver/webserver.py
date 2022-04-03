import argparse
from flask import Flask, render_template, send_from_directory
from flask_restful import Api
from config_data import Config, ConfigController, ConfigSetpoint, set_config_filepath
from calibration import Calibration, CalibrateGyroOutput

app = Flask(__name__, static_url_path='', static_folder='static')
api = Api(app)


@app.route('/', methods=['GET'])
def index():
    return render_template('index.html')


@app.route('/calibration.html', methods=['GET'])
def calibration_ui():
    return render_template('calibration.html')


@app.route('/config_controller.html', methods=['GET'])
def config_controller():
    return render_template('config_controller.html')


@app.route('/config_setpoint.html', methods=['GET'])
def config_setpoint():
    return render_template('config_setpoint.html')


@app.route('/static/<path:path>')
def send_js(path):
    return send_from_directory('js', path)


api.add_resource(ConfigController, '/config_controller_app')
api.add_resource(ConfigSetpoint, '/config_setpoint_app')
api.add_resource(Config, '/config')
api.add_resource(Calibration, '/calibration')
api.add_resource(CalibrateGyroOutput, '/calibrate_gyro_output')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument('-c',
                        '--config',
                        help='Path to Config File',
                        required=False)
    args = vars(parser.parse_args())

    print(f'args {args}')

    if "config" in args and args['config'] is not None:
        set_config_filepath(args['config'])

    app.run(debug=True, host='0.0.0.0', port=5001)  # run our Flask app

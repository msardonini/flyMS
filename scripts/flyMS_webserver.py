
from flask import Flask
from flask_restful import Resource, Api, reqparse
# import pandas as pd
import ast
import yaml

app = Flask(__name__)
api = Api(app)

class Config(Resource):
    def get(self):
        with open('/home/debian/.config/flyMSConfig.yaml', "r") as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return {'data': data}, 200  # return data and 200 OK code


api.add_resource(Config, '/config')

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, port=5001)  # run our Flask app

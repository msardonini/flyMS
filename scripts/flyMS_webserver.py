
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
                print(data)
            except yaml.YAMLError as exc:
                print(exc)


        # data = data.to_dict()  # convert dataframe to dictionary

        return {'data': data}, 200  # return data and 200 OK code


api.add_resource(Config, '/config')  # '/users' is our entry point for Users

if __name__ == '__main__':
    app.run(debug=True, port=5001)  # run our Flask app

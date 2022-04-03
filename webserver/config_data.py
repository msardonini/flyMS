from dataclasses import asdict
from flask import request
from flask_restful import Resource
from flyMSConfig import flyMSConfig
import dacite
import yaml
import io

# The configuration file's default value
config_filepath = '/home/debian/.config/flyMSConfig.yaml'


def set_config_filepath(filename_local: str):
    global config_filepath
    config_filepath = filename_local
    return


def count_keys(dict_, counter=0):
    for each_key in dict_:
        if isinstance(dict_[each_key], dict):
            # Recursive call
            counter = count_keys(dict_[each_key], counter + 1)
        else:
            counter += 1
    return counter


def apply_overrides(conf: flyMSConfig, overrides: dict):
    for key, val in overrides.items():
        # If the val is another dictionary, recursively iterate
        if type(val) is dict:
            apply_overrides(getattr(conf, key), val)
        else:
            tmp_type = type(getattr(conf, key))
            # If our val is a list, apply type conversions on each element
            if tmp_type is list:
                if len(getattr(conf, key)) == 0:
                    raise RuntimeError("Received zero length list!")
                derired_type = type(getattr(conf, key)[0])
                tmp = [derired_type(x) for x in val]
            else:
                derired_type = type(getattr(conf, key))
                tmp = derired_type(val)
            setattr(conf, key, tmp)


class Config(Resource):

    def get(self):
        with open(config_filepath, 'r') as stream:
            data = yaml.safe_load(stream)
        # print(f'the data is {data}')
        return data  # return data and 200 OK code


class ConfigController(Resource):

    def get(self):
        with open(config_filepath, 'r') as stream:
            data = yaml.safe_load(stream)
        # print(f'the data is {data}')
        return data['flyMSParams']['controller']  # return data and 200 OK code

    def post(self):
        with open(config_filepath, 'r') as stream:
            orig_data = yaml.safe_load(stream)

        dataclass_obj = dacite.from_dict(data_class=flyMSConfig,
                                         data=orig_data['flyMSParams'])
        apply_overrides(dataclass_obj.controller, request.json)

        with io.open(config_filepath, 'w', encoding='utf8') as outfile:
            output_dict = {'flyMSParams': asdict(dataclass_obj)}
            yaml.dump(output_dict, outfile)


class ConfigSetpoint(Resource):

    def get(self):
        with open(config_filepath, 'r') as stream:
            data = yaml.safe_load(stream)
        return data['flyMSParams']['setpoint']  # return data and 200 OK code

    def post(self):
        with open(config_filepath, 'r') as stream:
            orig_data = yaml.safe_load(stream)

        dataclass_obj = dacite.from_dict(data_class=flyMSConfig,
                                         data=orig_data['flyMSParams'])
        apply_overrides(dataclass_obj.setpoint, request.json)

        with io.open(config_filepath, 'w', encoding='utf8') as outfile:
            output_dict = {'flyMSParams': asdict(dataclass_obj)}
            yaml.dump(output_dict, outfile)

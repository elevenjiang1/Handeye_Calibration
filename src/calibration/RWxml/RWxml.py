#!/usr/bin/env python

import yaml
import os
import xml.etree.ElementTree as ET
import numpy as np

filefolder = os.path.dirname(__file__)
filename_yaml = os.path.join(filefolder, '..', 'calibration_files', 'transforms.yaml')
filename_xml = os.path.join(filefolder, '..', 'calibration_files', 'camera_info.xml')
with open(filename_yaml, 'r') as yaml_file:
    data = yaml.load(yaml_file)
K = np.array(data['camera_matrix']['data']).reshape(3, 3)
D = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
image_width = data['image_width']
image_height = data['image_height']

tree = ET.parse(filename_xml)
root = tree.getroot()

for param in root:
    if param.get('name') == 'ResolutionRGB':
        param[0].text = str(image_width)
        param[1].text = str(image_height)
    elif param.get('name') == 'FocalLengthRGB':
        param[0].text = str(K[0,0])
        param[1].text = str(K[1,1])
    elif param.get('name') == 'PrincipalPointRGB':
        param[0].text = str(K[0,2])
        param[1].text = str(K[1,2])
    elif param.get('name') == 'DistortionRGB':
        param[0].text = str(D[0,0])
        param[1].text = str(D[0,1])
        param[2].text = str(D[0,2])
        param[3].text = str(D[0,3])
        param[4].text = str(D[0,4])
tree.write(filename_xml)
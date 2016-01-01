'''
Created on Jan 1, 2016

@author: Gregory Kramida
'''
from lxml import etree
import numpy as np
import calib.geom as geom
import re


def load_corners(path, board_height = None, board_width = None, board_square_size = None):
    npzfile = np.load(path)
    limgpoints = npzfile['limgpoints']
    rimgpoints = npzfile['rimgpoints']
    if 'objp' in npzfile:
        objp = npzfile['objp']
    else:
        objp = geom.generate_object_points(board_height, board_width, board_square_size)
    objpoints = []
    usable_frame_ct = len(limgpoints)
    for i_frame in xrange(usable_frame_ct): # @UnusedVariable
        objpoints.append(objp)
    return limgpoints,rimgpoints,objpoints, usable_frame_ct

def parse_xml_matrix(mat_element):
    '''
    Generate numpy matrix from opencv-formatted xml of a 2d matrix
    '''
    rows = int(mat_element.find("rows").text)
    cols = int(mat_element.find("cols").text)
    type_flag = mat_element.find("dt").text
    if type_flag == "f":
        dtype = np.float32
    elif type_flag == "d":
        dtype = np.float64
    else:
        raise ValueError("dtype flag " + type_flag + " not supported." )
    data_string = mat_element.find("data").text
    data = np.array([float(part) for part in data_string.strip().split(" ") if len(part) > 0])
    return data.reshape((rows,cols)).astype(dtype)

def load_opencv_calibration(path):
    tree = etree.parse(path)
    error = float(tree.find("reprojection_error").text)
    K1 = parse_xml_matrix(tree.find("K1"))
    d1 = parse_xml_matrix(tree.find("d1"))
    K2 = parse_xml_matrix(tree.find("K2"))
    d2 = parse_xml_matrix(tree.find("d2"))
    R = parse_xml_matrix(tree.find("R"))
    T = parse_xml_matrix(tree.find("T"))
    width = float(tree.find("width").text)
    height = float(tree.find("height").text)
    return error, K1, d1, K2, d2, R, T, (height,width)   

def make_opencv_matrix_xml_element(root, mat,name):
    mat_element = etree.SubElement(root, name, attrib={"type_id":"opencv-matrix"})
    rows_elem = etree.SubElement(mat_element, "rows")
    rows_elem.text = str(mat.shape[0])
    cols_elem = etree.SubElement(mat_element, "cols")
    cols_elem.text = str(mat.shape[1])
    dt_elem = etree.SubElement(mat_element, "dt")
    if(mat.dtype == np.dtype('float64')):
        dt_elem.text = "d"
    elif(mat.dtype == np.dtype("float32")):
        dt_elem.text = "f"
    else:
        raise ValueError("dtype " + str(mat.dtype) + "not supported")
    
    data_elem = etree.SubElement(mat_element, "data")
    data_string = str(mat.flatten()).replace("\n","").replace("[","").replace("]","")
    data_string = re.sub("\s+"," ",data_string)
    data_elem.text = data_string
    return mat_element
    
def save_output_opencv(error, K1, d1, K2, d2, R, T, dims, file_path):
    root = etree.Element("opencv_storage")
    width_element = etree.SubElement(root, "width")
    width_element.text = str(dims[1])
    width_element = etree.SubElement(root, "height")
    width_element.text = str(dims[0])
    K1_element = make_opencv_matrix_xml_element(root, K1, "K1")  # @UnusedVariable
    d1_element = make_opencv_matrix_xml_element(root, d1, "d1") # @UnusedVariable
    K2_element = make_opencv_matrix_xml_element(root, K2, "K2") # @UnusedVariable
    d2_element = make_opencv_matrix_xml_element(root, d2, "d2") # @UnusedVariable
    R_element = make_opencv_matrix_xml_element(root, R, "R") # @UnusedVariable
    T_element = make_opencv_matrix_xml_element(root, T, "T") # @UnusedVariable
    error_element = etree.SubElement(root, "reprojection_error")
    error_element.text = str(error)
    et = etree.ElementTree(root)
    with open(file_path,'wb') as f:
        et.write(f,encoding="utf-8",xml_declaration=True, pretty_print=True)
    s=open(file_path).read()
    s = s.replace("'","\"")
    with open(file_path,'w') as f:
        f.write(s)
        f.flush()
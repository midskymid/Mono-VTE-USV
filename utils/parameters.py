"""
Authors: Liu Zhongtian (midsky@zju.edu.cn); Chen Xuanlin (xuanlinchen@zju.edu.cn)
"""
import cv2
import os


def readParameters(path: str) -> dict:
    """read parameters from config file and return dictionary of parameters
       Input: type(str) : your config file which format is OpenCV YAML.
       Output: type(dict) : MONO-VTE-USB related parameters.
    """
    params = {}
    
    if not os.path.exists(path):
        raise FileNotFoundError(f"config file not found at {path}, please check the config file path...")
    if not path.lower().endswith('.yaml'):
        raise ValueError(f"{path} is not YAML format...")
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    
        params['PLUGIN_LIBRARY'] = fs.getNode('PLUGIN_LIBRARY').string()
        params['engine_file_path'] = fs.getNode('engine_file_path').string()
        params['connect'] = fs.getNode('connect').string()
        params['log_path'] = fs.getNode('log_path').string()

        params['baud'] = int(fs.getNode('baud').real())
        params['K1'] = fs.getNode('K1').real()
        params['K2'] = fs.getNode('K2').real()
        params['K3'] = fs.getNode('K3').real()
        params['P1'] = fs.getNode('P1').real()
        params['P2'] = fs.getNode('P2').real()
        params['omega'] = fs.getNode('omega').real()
        params['yita'] = fs.getNode('yita').real()
        params['deta'] = fs.getNode('deta').real()
        params['OME'] = fs.getNode('OME').real()
          
        params['distortion_parameters'] = [fs.getNode('distortion_parameters').getNode(key).real() for key in fs.getNode('distortion_parameters').keys()]
        params['projection_parameters'] = [fs.getNode('projection_parameters').getNode(key).real() for key in fs.getNode('projection_parameters').keys()]
        
        return params
          
    except Exception as e:
        print("Error loading YAML: ", e)
        
        
if __name__ == "__main__":
    # change your config file path to check
    path = "your_config_yaml_file_path.yaml"
    params = readParameters(path)
    print("The paramters are as follows: \n", params)
    
        

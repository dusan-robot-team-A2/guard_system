import numpy as np
import cv2

class VIPManagementSystem:
    def __init__(self):
        pass

    def add_vip(self, np_dic):
        if np.load('vip_lst'):
            vip_lst = np.load('vip_lst')
            np.savez('vip_lst', **vip_lst, **np_dic)
        else:
            np.savez('vip_lst', **np_dic)
    
    def search_vip(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        np_image = np.array(image)
        vip_dic = np.load('vip_lst')
        vip_lst = []
        for k, v in vip_dic.items():
            vip_lst.append(v)
        if np_image in vip_lst:
            return True
        else:
            return False
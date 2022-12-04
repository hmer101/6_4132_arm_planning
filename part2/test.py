import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), 'padm-project=2022f', d)) for d in ['pddlstream', 'ss-pybullet','ss-pybullet/pybullet_tools'])
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['','.gitmodules'])
__import__("padm-project-2022f")
#import ss-pybullet
#__import__(os.path.join(os.getcwd(),'padm-project-2022f')) 

#from pybullet_tools.utils import set_joint_positions

if __name__ == '__main__':
    print(sys.path)


from object_definitions import get_OBJ
from object_surf_func import plot_3d_object

if __name__=="__main__":
    OBJ = get_OBJ("rectangular_prism_small",3)
    plot_3d_object(OBJ)
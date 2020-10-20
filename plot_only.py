from object_definitions import get_OBJ
from object_surf_func import plot_3d_object

if __name__=="__main__":
    OBJ = get_OBJ("square_prism",2)
    plot_3d_object(OBJ)
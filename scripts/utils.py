"""Helpers
"""

import os
import math

def subsample_point_cloud_txt(file_in, file_out = ''):
    """ Subsample point cloud data.
    
    inputs:
    file_in: Input file path. Assumed to be txt file. 
    file_out: Output file path. Defaults to file_in_subsampled.

    returns: 
    """

    max_file_size = 30 # in MB
    statinfo = os.stat(file_in)
    file_in_size = statinfo.st_size*1e-6

    if file_in_size < max_file_size:
        print 'Input file size: {} is less than max file size: {}.\n'.format(file_in_size, max_file_size)
    
    subsample_factor = math.ceil(file_in_size/max_file_size)
    
    file_in_no_ext = os.path.splitext(file_in)[0]
    if file_out == '':
        file_out = file_in_no_ext + '_subsampled.txt'
    file_out_obj = open(file_out, 'w')


    print 'Writing to {}.\n'.format(file_out),
    count = 0
    with open(file_in, 'r') as file_in_obj:
        for line in file_in_obj:
            if (count % subsample_factor) == 0:
                file_out_obj.write(line)
            count = count+1

    file_out_obj.close()

def gen_pcd_header(num_pts):
    """ Generate pcd header. Assumes xyzrgb, unorganized.
    
    inputs:
    num_pts: Number of points

    returns: 
    """
    content = '# .PCD v.7 - Point Cloud Data file format\n' \
           'VERSION .7\n' \
           'FIELDS x y z rgb\n' \
           'SIZE 4 4 4 4\n' \
           'TYPE F F F F\n' \
           'COUNT 1 1 1 1\n' \
           'WIDTH {}\n' \
           'HEIGHT 1\n' \
           'VIEWPOINT 0 0 0 1 0 0 0\n' \
           'POINTS {}\n' \
           'DATA ascii'.format(num_pts, num_pts)
    return content

def append_pcd_header_to_point_cloud_txt(file_in, file_out = ''):
    """ Append header needed by pcl.
    
    inputs:
    file_in: Input file path. Assumed to be txt file. 
    file_out: Output file path. Defaults to file_in.pcd

    returns: 
    """

    # get number of points
    with open(file_in) as f:
        for num_pts, l in enumerate(f):
            pass

    pcd_header = gen_pcd_header(num_pts)
            
    if file_out == '':
        file_out = os.path.splitext(file_in)[0] + '.pcd'
        
    file_out_obj = open(file_out, 'w')
    print 'Writing to {}.\n'.format(file_out),
    file_out_obj.write(pcd_header)
    file_out_obj.write('\n')

    file_in_obj = open(file_in, 'r')
    contents = file_in_obj.read()
    file_out_obj.write(contents)

    file_in_obj.close()
    file_out_obj.close()
    


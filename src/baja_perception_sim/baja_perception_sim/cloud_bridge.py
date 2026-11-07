"""Reference cloud converter for tests; production bridge is in baja_gz_bridge."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def convert(msg):
    fields={f.name:f for f in msg.field}
    names=['x','y','z','intensity']
    if not all(n in fields for n in names):
        raise ValueError('Gazebo cloud is missing XYZ/intensity')
    # Gazebo 8 may advertise its float bytes as UINT32. Read known fields as float.
    dt=np.dtype({'names':names,'formats':['>f4' if msg.is_bigendian else '<f4']*4,
                 'offsets':[fields[n].offset for n in names],'itemsize':msg.point_step})
    a=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    dtype=np.dtype({'names':names+['ring'],'formats':['<f4']*4+['<u2'],
                    'offsets':[0,4,8,12,16],'itemsize':20})
    data=np.zeros(a.shape,dtype=dtype)
    for name in names: data[name]=a[name]
    # Gazebo organized scan rows run from vertical minimum to maximum.
    data['ring']=np.arange(msg.height,dtype=np.uint16)[:,None]
    out=PointCloud2()
    out.header.stamp.sec=msg.header.stamp.sec
    out.header.stamp.nanosec=msg.header.stamp.nsec
    out.header.frame_id='lidar_link'
    out.height=msg.height;out.width=msg.width;out.point_step=20;out.row_step=20*msg.width
    out.is_bigendian=False;out.is_dense=False
    out.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(names)]
    out.fields.append(PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1))
    out.data=data.tobytes()
    return out

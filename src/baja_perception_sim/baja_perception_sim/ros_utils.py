import copy
import math

import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import to_local


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_key(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def pose_arrays(pose):
    p, q = pose.position, pose.orientation
    return np.array([p.x,p.y,p.z]), np.array([q.x,q.y,q.z,q.w])


def cloud_xyz(msg: PointCloud2):
    offsets = {f.name:f.offset for f in msg.fields}
    if not all(n in offsets for n in ['x','y','z']):
        raise ValueError('Point cloud must have x/y/z fields')
    dt = np.dtype({'names':['x','y','z'], 'formats':[('>f4' if msg.is_bigendian else '<f4')]*3,
                   'offsets':[offsets[n] for n in ['x','y','z']], 'itemsize':msg.point_step})
    a = np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    points = np.stack([a[n].ravel() for n in ['x','y','z']],axis=1)
    return points[np.isfinite(points).all(axis=1)]


def detection_array(objects, header, pose=None):
    out=Detection3DArray(header=copy.deepcopy(header))
    for obj in objects:
        det=Detection3D(header=copy.deepcopy(header),id=obj['id'])
        p=np.array(obj['position'])
        rot=Rotation.from_euler('z',obj['yaw'])
        if pose is not None:
            p=to_local([p],*pose)[0]
            rot=Rotation.from_quat(pose[1]).inv()*rot
        det.bbox.center.position.x,det.bbox.center.position.y,det.bbox.center.position.z=map(float,p)
        q=rot.as_quat()
        det.bbox.center.orientation.x,det.bbox.center.orientation.y,det.bbox.center.orientation.z,det.bbox.center.orientation.w=map(float,q)
        det.bbox.size.x,det.bbox.size.y,det.bbox.size.z=map(float,obj['size'])
        hypothesis=ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id=obj['class_id']
        hypothesis.hypothesis.score=1.0
        hypothesis.pose.pose=copy.deepcopy(det.bbox.center)
        det.results=[hypothesis]
        out.detections.append(det)
    return out


def detection_markers(msg, namespace, color):
    result=MarkerArray()
    clear=Marker();clear.action=Marker.DELETEALL;clear.ns=namespace
    result.markers.append(clear)
    for i,d in enumerate(msg.detections):
        marker=Marker(header=msg.header,ns=namespace,id=2*i,type=Marker.CUBE,action=Marker.ADD)
        marker.pose=d.bbox.center;marker.scale=d.bbox.size
        marker.color.r,marker.color.g,marker.color.b,marker.color.a=color
        result.markers.append(marker)
        text=copy.deepcopy(marker);text.id+=1;text.type=Marker.TEXT_VIEW_FACING
        text.pose.position.z+=d.bbox.size.z/2+0.25
        text.scale.x=0.;text.scale.y=0.;text.scale.z=0.24;text.color.a=1.
        text.text=d.results[0].hypothesis.class_id if d.results else 'unknown'
        result.markers.append(text)
    return result


def transform(parent,child,translation,quaternion,stamp):
    out=TransformStamped()
    out.header.frame_id=parent;out.child_frame_id=child;out.header.stamp=stamp
    out.transform.translation.x,out.transform.translation.y,out.transform.translation.z=map(float,translation)
    out.transform.rotation.x,out.transform.rotation.y,out.transform.rotation.z,out.transform.rotation.w=map(float,quaternion)
    return out


def boundary_markers(msg, namespace='boundaries'):
    out=MarkerArray()
    clear=Marker();clear.action=Marker.DELETEALL;out.markers.append(clear)
    for i,b in enumerate(msg.boundaries):
        m=Marker(header=msg.header,ns=namespace,id=i,type=Marker.LINE_STRIP,action=Marker.ADD)
        m.pose.orientation.w=1.;m.scale.x=0.08;m.color.a=1.
        m.color.b=1. if b.side==1 else 0.;m.color.r=1. if b.side==2 else 0.
        m.points=b.points;out.markers.append(m)
    return out

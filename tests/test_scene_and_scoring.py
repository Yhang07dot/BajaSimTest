import copy
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml
from scipy.spatial import cKDTree

from baja_perception_sim.scene import generate, height
from baja_perception_sim.geometry import (track_length, track_pose, track_samples,
                                         PoseHistory, match_centres, points_in_box,
                                         polyline_distances)
from baja_perception_sim.geometry import pursuit_yaw_rate
from baja_perception_sim.evaluation import score_frame, summarize
from baja_perception_sim.ros_utils import cloud_xyz
from sensor_msgs.msg import PointCloud2, PointField

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'src/baja_perception_sim'


@pytest.fixture
def cfg():
    return yaml.safe_load((PKG/'config/scenario.yaml').read_text())


def test_track_closes_without_duplicate_endpoint(cfg):
    t=cfg['track'];length=track_length(t)
    assert track_pose(length,t)==track_pose(0,t)
    p=track_samples(t)
    assert np.linalg.norm(p[-1,:2]-p[0,:2])<=t['sample_step']+1e-6
    assert np.linalg.norm(p[-1,:2]-p[0,:2])>0


def test_deterministic_scene_full_curve_clearance_and_real_mesh(tmp_path,cfg):
    a=generate(cfg,tmp_path/'a',PKG/'models/vehicle.sdf')
    b=generate(cfg,tmp_path/'b',PKG/'models/vehicle.sdf')
    assert a==b
    other=copy.deepcopy(cfg);other['seed']=7
    c=generate(other,tmp_path/'c',PKG/'models/vehicle.sdf')
    assert a['objects']!=c['objects']
    # Independently use denser sampling than generation's safety check.
    t=copy.deepcopy(cfg['track']);t['sample_step']=0.025
    tree=cKDTree(track_samples(t)[:,:2])
    for obj in a['objects']+c['objects']:
        distance=tree.query(obj['position'][:2])[0]
        assert distance-math.hypot(*obj['size'][:2])/2>=cfg['vehicle']['clear_radius']
    sdf=ET.parse(tmp_path/'a/world.sdf')
    assert not any(m.get('name','').startswith('parking') for m in sdf.findall('.//model'))
    assert not any(r['class_id']=='parking_bay' for r in a['regions'])
    sensor=sdf.find(".//sensor[@name='lidar']")
    assert sensor.findtext('lidar/scan/vertical/samples')=='16'
    assert sensor.findtext('lidar/scan/horizontal/samples')=='2000'
    assert sensor.findtext('lidar/scan/vertical/resolution')=='1'
    assert sensor.findtext('lidar/scan/horizontal/resolution')=='1'
    road=(tmp_path/'a/meshes/road.obj').read_text()
    assert 'vn ' in road and '//' in road  # DART requires normals on collisions.
    assert height(18,0,cfg)==-cfg['terrain']['crater_depth']
    assert height(26,0,cfg)==cfg['terrain']['bump_height']


def test_unsafe_corridor_rejected(tmp_path,cfg):
    cfg['vehicle']['clear_radius']=0.5
    with pytest.raises(ValueError,match='diagonal'):
        generate(cfg,tmp_path,PKG/'models/vehicle.sdf')


def test_high_speed_configuration_matches_physical_limiter(tmp_path,cfg):
    cfg['vehicle']['speed'] = 3.0
    generate(cfg,tmp_path,PKG/'models/vehicle.sdf')
    sdf=ET.parse(tmp_path/'world.sdf')
    assert float(sdf.findtext(".//plugin[@name='gz::sim::systems::AckermannSteering']/max_velocity")) == cfg['vehicle']['max_speed']
    cfg['vehicle']['speed'] = 3.1
    with pytest.raises(ValueError,match='vehicle.speed'):
        generate(cfg,tmp_path,PKG/'models/vehicle.sdf')


def test_steering_authority_scales_with_speed():
    target = [2.5,0.6,0.0]
    rates = [pursuit_yaw_rate(v,target) for v in [1.,2.,3.]]
    assert np.allclose(rates,np.array([1.,2.,3.])*rates[0])
    assert rates[-1] > 0.3
    assert abs(pursuit_yaw_rate(3.,[0,1,0])) <= 3*math.tan(math.radians(26))/1.43


def test_pose_interpolation_uses_short_rotation():
    h=PoseHistory()
    h.add(0,[0,0,0],[0,0,math.sin(math.radians(179)/2),math.cos(math.radians(179)/2)])
    h.add(2,[2,0,0],[0,0,math.sin(math.radians(-179)/2),math.cos(math.radians(-179)/2)])
    p,q=h.at(1)
    assert np.allclose(p,[1,0,0])
    assert abs(q[2])>0.999
    assert h.at(10) is None
    h.add(0,[0,0,0],[0,0,0,1])
    assert len(h.items)==1


def test_matching_prioritizes_cardinality():
    matches=match_centres([[0,0,0],[1,0,0]],[[0.4,0,0],[-0.5,0,0]],0.7)
    assert len(matches)==2


def obj(x,cls='box',ident='truth'):
    return {'position':[x,4,0.5],'size':[1,1,1],'class_id':cls,'id':ident,'score':0.9}


def test_scoring_duplicates_misclassification_and_invisible(cfg):
    e=cfg['evaluation']
    result=score_frame([obj(10),obj(10.1)], [obj(10)], [obj(10)], e)
    assert result['objectness']==(1,1,0)
    result=score_frame([obj(10,'person')], [obj(10)], [obj(10)], e)
    assert result['objectness']==(1,0,0)
    assert result['semantic']==(0,1,1)
    result=score_frame([obj(10)], [], [obj(10)], e)
    assert result['objectness']==(0,0,0)
    assert result['ignored_predictions']==1
    result=score_frame([], [obj(10)], [obj(10)], e)
    assert result['objectness']==(0,0,1)
    assert summarize(0,0,0)['score_100'] is None


def test_finite_polyline_does_not_extrapolate():
    d=polyline_distances([[0,0],[3,0]],[[[0,0],[1,0]]])
    assert np.allclose(d,[0,2])


def test_oriented_box_visibility():
    o={'position':[1,1,0],'size':[4,0.2,1],'yaw':math.pi/2}
    assert points_in_box([[1,2,0],[2,1,0]],o).tolist()==[True,False]


@pytest.mark.parametrize('bigendian',[False,True])
def test_runtime_cloud_reader_handles_padding_and_invalid_returns(bigendian):
    msg=PointCloud2()
    msg.width=2;msg.height=2;msg.point_step=20;msg.row_step=64
    msg.is_bigendian=bigendian
    msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(['x','y','z','intensity'])]
    msg.fields.append(PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1))
    values=[[1,2,3],[4,5,6],[7,8,9],[float('inf'),0,0]]
    data=bytearray(msg.row_step*msg.height)
    for i,p in enumerate(values):
        offset=(i//2)*msg.row_step+(i%2)*msg.point_step
        struct.pack_into(('>' if bigendian else '<')+'fff',data,offset,*p)
    msg.data=bytes(data)
    assert np.allclose(cloud_xyz(msg),values[:3])

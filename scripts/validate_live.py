#!/usr/bin/env python3
"""Run in a sourced terminal. Starts its own simulator and writes a small report."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from PIL import Image as PillowImage
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from baja_perception_sim.ros_utils import cloud_xyz,seconds
from baja_perception_sim.geometry import track_samples
from scipy.spatial import cKDTree
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sim-seconds',type=float,default=110)
    parser.add_argument('--wall-timeout',type=float,default=360)
    parser.add_argument('--algorithm',default='')
    parser.add_argument('--speed',type=float)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=(args.output or root/'generated/validation').resolve();output.mkdir(parents=True,exist_ok=True)
    env=dict(os.environ,GZ_PARTITION='baja_validation',ROS_DOMAIN_ID='72')
    os.environ.update(GZ_PARTITION=env['GZ_PARTITION'],ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'])
    log=(output/'simulation.log').open('w')
    command=[str(root/'run.sh'),'--headless','--run-dir',str(output)]
    command+=['--algorithm',args.algorithm] if args.algorithm else ['--example']
    if args.speed is not None:command+=['--speed',str(args.speed)]
    process=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('live_validation')
    cfg=yaml.safe_load((root/'src/baja_perception_sim/config/scenario.yaml').read_text())
    if args.speed is not None:cfg['vehicle']['speed']=args.speed
    tree=cKDTree(track_samples(cfg['track'])[:,:2])
    counts={k:0 for k in ['cloud','image','camera_info','odom','summary','score_marker']}
    report={'counts':counts};cloud_times=[];poses=[];errors=[];latest={}

    def cloud(m):
        counts['cloud']+=1;cloud_times.append(seconds(m.header.stamp))
        p=cloud_xyz(m)
        report['cloud']={'width':m.width,'height':m.height,'finite_points':len(p),'frame':m.header.frame_id,'fields':[f.name for f in m.fields]}
        if len(p):
            angles=np.degrees(np.arctan2(p[:,2],np.hypot(p[:,0],p[:,1])))
            report['cloud']['elevation_deg']=[float(angles.min()),float(angles.max())]
        latest['cloud']=m

    def image(m):counts['image']+=1;latest['image']=m
    def info(m):
        counts['camera_info']+=1
        report['camera_info']={'frame':m.header.frame_id,'width':m.width,'height':m.height,'fx':m.k[0]}
    def odom(m):
        counts['odom']+=1;p=m.pose.pose.position
        poses.append([seconds(m.header.stamp),p.x,p.y,p.z])
        errors.append(float(tree.query([p.x,p.y])[0]))
    def summary(m):counts['summary']+=1;report['evaluation']=json.loads(m.data)
    def score_marker(m):counts['score_marker']+=1;report['score_marker_text']=m.text
    node.create_subscription(PointCloud2,'/sensors/lidar/points',cloud,qos_profile_sensor_data)
    node.create_subscription(Image,'/sensors/front_camera/image_raw',image,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/sensors/front_camera/camera_info',info,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/ground_truth/odom',odom,10)
    node.create_subscription(String,'/evaluation/summary',summary,10)
    node.create_subscription(Marker,'/evaluation/score_marker',score_marker,10)
    started=time.monotonic();last_print=started
    try:
        while time.monotonic()-started<args.wall_timeout and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=0.1)
            if time.monotonic()-last_print>15:
                print(json.dumps({'counts':counts,'last_pose':poses[-1] if poses else None,'max_error':max(errors) if errors else None}),flush=True)
                last_print=time.monotonic()
            if poses and poses[-1][0]>=args.sim_seconds:break
        if poses:
            a=np.array(poses)
            report['motion']={'sim_seconds':float(a[-1,0]),'distance_m':float(np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1).sum()),
                              'max_cross_track_m':max(errors),'mean_cross_track_m':float(np.mean(errors)),
                              'start':poses[0],'end':poses[-1],'min_base_z':float(a[:,3].min()),'max_base_z':float(a[:,3].max()),
                              'target_speed_mps':cfg['vehicle']['speed']}
            np.savez_compressed(output/'trajectory.npz',poses=a)
        if len(cloud_times)>1:report['cloud_sim_hz']=(len(cloud_times)-1)/(cloud_times[-1]-cloud_times[0])
        if 'image' in latest:
            m=latest['image'];data=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
            if m.encoding=='bgr8':data=data[:,:,::-1]
            PillowImage.fromarray(data).save(output/'front_camera.jpg',quality=85)
            report['image']={'width':m.width,'height':m.height,'encoding':m.encoding,'frame':m.header.frame_id}
        if 'cloud' in latest:
            np.savez_compressed(output/'one_cloud.npz',xyz=cloud_xyz(latest['cloud']))
        checks={
            'all_streams':all(counts.values()),
            'cloud_shape':report.get('cloud',{}).get('height')==16 and report.get('cloud',{}).get('width')==2000,
            'finite_returns':report.get('cloud',{}).get('finite_points',0)>100,
            'camera_frame':report.get('image',{}).get('frame')=='camera_optical_frame',
            'camera_calibrated':report.get('camera_info',{}).get('fx',0)>0,
            'motion':report.get('motion',{}).get('distance_m',0)>args.sim_seconds*cfg['vehicle']['speed']*0.8,
            'tracking':report.get('motion',{}).get('max_cross_track_m',999)<0.65,
            'completed_duration':report.get('motion',{}).get('sim_seconds',0)>=args.sim_seconds-0.1,
            'score_active':report.get('evaluation',{}).get('status')=='running',
        }
        if not args.algorithm or args.algorithm=='baja_algorithm_template':
            checks['example_zero_tp']=report.get('evaluation',{}).get('objectness',{}).get('tp')==0
        checks={k:bool(v) for k,v in checks.items()}
        report['checks']=checks
        (output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        # Signal launch only; it propagates once to children. Signalling the whole
        # group as well interrupts Python cleanup twice and can kill native callbacks.
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        log.close();node.destroy_node();rclpy.try_shutdown()
    if not all(checks.values()):raise SystemExit(1)


if __name__=='__main__':main()

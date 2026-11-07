"""Timestamp-aligned LiDAR-visible object evaluation; no fabricated detections."""
import json
import math
from pathlib import Path
from collections import Counter, OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker
from baja_sim_interfaces.msg import RoadBoundaryArray

from .geometry import match_centres, polyline_distances, clip_polylines, sample_polylines
from .ros_utils import pose_arrays, seconds, stamp_key
from .score_display import marker_text


def summarize(tp,fp,fn):
    precision=tp/(tp+fp) if tp+fp else None
    recall=tp/(tp+fn) if tp+fn else None
    f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else None
    return {'tp':tp,'fp':fp,'fn':fn,'precision':precision,'recall':recall,'f1':f1,
            'score_100':None if f1 is None else 100*f1,
            'false_positive_penalty_100':100*fp/(2*tp+fp+fn) if 2*tp+fp+fn else None,
            'false_negative_penalty_100':100*fn/(2*tp+fp+fn) if 2*tp+fp+fn else None}


def boundary_records(msg):
    return [{'side':b.side,'confidence':b.confidence,
             'points':[[p.x,p.y] for p in b.points]} for b in msg.boundaries]


def score_boundary_frame(pred, truth, cfg):
    result = {'truth_points':0,'pred_points':0,'covered':0,'correct':0,'error_sum':0.}
    tolerance = cfg.get('boundary_tolerance',0.5)
    step = cfg.get('boundary_sample_step',0.5)
    for side in [1,2]:
        def lines(records):
            return clip_polylines([b['points'] for b in records if b['side']==side and
                                   b['confidence']>=cfg['confidence_threshold']],
                                  cfg['min_forward'],cfg['max_distance'])
        gt, estimate = lines(truth), lines(pred)
        gt_points, pred_points = sample_polylines(gt,step), sample_polylines(estimate,step)
        recall_distances = polyline_distances(gt_points,estimate)
        precision_distances = polyline_distances(pred_points,gt)
        covered, correct = recall_distances<=tolerance, precision_distances<=tolerance
        result['truth_points'] += len(gt_points)
        result['pred_points'] += len(pred_points)
        result['covered'] += int(covered.sum())
        result['correct'] += int(correct.sum())
        result['error_sum'] += float(recall_distances[covered].sum())
    return result


def boundary_summary(truth_points,pred_points,covered,correct):
    precision = correct/pred_points if pred_points else None
    recall = covered/truth_points if truth_points else None
    if not truth_points and not pred_points:
        f1 = None
    elif not precision or not recall:
        f1 = 0.
    else:
        f1 = 2*precision*recall/(precision+recall)
    return {'precision':precision,'recall':recall,'f1':f1,
            'score_100':100*f1 if f1 is not None else None}


def unpack(msg):
    return [{'id':d.id,'class_id':d.results[0].hypothesis.class_id,
             'score':d.results[0].hypothesis.score,'position':pose_arrays(d.bbox.center)[0].tolist(),
             'size':[d.bbox.size.x,d.bbox.size.y,d.bbox.size.z]} for d in msg.detections]


def score_frame(pred,truth,roi,cfg,boundary_lines=None):
    pred=[p for p in pred if p['score']>=cfg['confidence_threshold'] and
          p['position'][0]>=cfg['min_forward'] and
          math.hypot(*p['position'][:2])<=cfg['max_distance'] and
          abs(math.atan2(p['position'][1],p['position'][0]))<=math.radians(cfg['half_fov_deg'])]
    # Boundary-cone detections must not accidentally match a nearby target under
    # the comparatively loose 1m centre gate. Target centres are placed outside
    # this narrow boundary background band by the scenario generator.
    background=set()
    if pred and boundary_lines:
        distances=polyline_distances([p['position'][:2] for p in pred],boundary_lines)
        background=set(np.flatnonzero(distances<=cfg.get('boundary_obstacle_band',0.25)).tolist())
    candidates=[i for i in range(len(pred)) if i not in background]
    matches=[(candidates[i],j,d) for i,j,d in match_centres(
        [pred[i]['position'] for i in candidates],[t['position'] for t in truth],cfg['match_distance'])]
    hit={i for i,_,_ in matches}
    visible_ids={t['id'] for t in truth}
    ignored=[t for t in roi if t['id'] not in visible_ids]
    # Detections of actual but LiDAR-invisible objects do not count as false positives.
    ignored_pred={i for i,p in enumerate(pred) if i not in hit and i not in background and any(
        np.linalg.norm(np.array(p['position'])-t['position'])<=cfg['match_distance'] for t in ignored)}
    fp=len(pred)-len(matches)-len(ignored_pred)
    classes={k:Counter() for k in {o['class_id'] for o in pred+truth}}
    for t in truth:classes[t['class_id']]['fn']+=1
    for i,p in enumerate(pred):
        if i not in ignored_pred:classes[p['class_id']]['fp']+=1
    correct=0
    for i,j,_ in matches:
        if pred[i]['class_id']==truth[j]['class_id']:
            c=classes[truth[j]['class_id']];c['tp']+=1;c['fn']-=1;c['fp']-=1;correct+=1
    return {'objectness':(len(matches),fp,len(truth)-len(matches)),
            'semantic':(correct,len(pred)-len(ignored_pred)-correct,len(truth)-correct),
            'classes':classes,'errors':[d for _,_,d in matches],
            'size_errors':[float(np.mean(np.abs(np.array(pred[i]['size'])-truth[j]['size']))) for i,j,_ in matches],
            'ignored_predictions':len(ignored_pred),'boundary_band_false_positives':len(background)}


class Evaluator(Node):
    def __init__(self):
        super().__init__('perception_evaluator')
        self.declare_parameter('scene_file','');self.declare_parameter('results_dir','results')
        scene=json.loads(Path(self.get_parameter('scene_file').value).read_text())
        self.cfg=scene['config']['evaluation'];self.seed=scene['config']['seed']
        self.output=Path(self.get_parameter('results_dir').value);self.output.mkdir(parents=True,exist_ok=True)
        self.frames=OrderedDict();self.active=False;self.start=None
        self.objectness=np.zeros(3,dtype=int);self.semantic=np.zeros(3,dtype=int)
        self.classes={};self.errors=[];self.size_errors=[];self.latencies=[]
        self.count=0;self.missing=0;self.invalid=0;self.late=0;self.ignored=0
        self.boundary_band_fp=0
        self.incomplete_truth_frames=0
        self.boundary_frames=0;self.boundary_points=0;self.boundary_covered=0;self.boundary_error=0.
        self.boundary_pred_points=0;self.boundary_correct=0
        self.boundary_active=False;self.boundary_start=None;self.boundary_missing=0
        self.pub=self.create_publisher(String,'/evaluation/summary',10)
        self.score_pub=self.create_publisher(Marker,'/evaluation/score_marker',10)
        self.get_logger().info(f'Score report: {self.output / "summary.json"}. View: ./score.sh or evaluation RViz LIVE SCORE.')
        self.create_subscription(Detection3DArray,'/ground_truth/eligible_objects',lambda m:self.receive(m,'truth'),10)
        self.create_subscription(Detection3DArray,'/ground_truth/roi_objects',lambda m:self.receive(m,'roi'),10)
        self.create_subscription(Detection3DArray,'/perception/objects_3d',lambda m:self.receive(m,'pred'),10)
        self.create_subscription(RoadBoundaryArray,'/ground_truth/road_boundaries',lambda m:self.boundary(m,'boundary_truth'),10)
        self.create_subscription(RoadBoundaryArray,'/perception/road_boundaries',lambda m:self.boundary(m,'boundary_pred'),10)
        self.create_timer(0.25,self.flush)
        self.create_timer(1.,self.report)

    def valid(self,msg):
        if msg.header.frame_id!='base_link':return False
        for d in msg.detections:
            if d.header.frame_id!=msg.header.frame_id or stamp_key(d.header.stamp)!=stamp_key(msg.header.stamp):return False
            p,q=pose_arrays(d.bbox.center)
            size=[d.bbox.size.x,d.bbox.size.y,d.bbox.size.z]
            if not d.results or not np.isfinite([*p,*q,*size]).all() or min(size)<=0 or abs(np.linalg.norm(q)-1)>0.01:return False
            h=d.results[0].hypothesis
            if not h.class_id or not math.isfinite(h.score) or not 0<=h.score<=1:return False
        return True

    def receive(self,msg,key):
        k=stamp_key(msg.header.stamp)
        now=seconds(self.get_clock().now().to_msg())
        if key=='pred':
            if not self.valid(msg):self.invalid+=1;return
            if now-k/1e9>self.cfg['result_timeout'] or now<k/1e9-0.05:self.late+=1;return
            if not self.active:self.active=True;self.start=k
        f=self.frames.setdefault(k,{})
        if key in f or f.get('done'):return
        f[key]=msg
        if key=='pred':f['latency']=now-k/1e9
        self.process(k,f)

    def process(self,k,f):
        if f.get('done') or not all(key in f for key in ['truth','roi','pred','boundary_truth']):return
        lines=[b['points'] for b in boundary_records(f['boundary_truth'])]
        result=score_frame(unpack(f['pred']),unpack(f['truth']),unpack(f['roi']),self.cfg,lines)
        self.objectness+=result['objectness'];self.semantic+=result['semantic']
        for cls,c in result['classes'].items():self.classes.setdefault(cls,Counter()).update(c)
        self.errors.extend(result['errors']);self.size_errors.extend(result['size_errors'])
        self.ignored+=result['ignored_predictions'];self.count+=1;f['done']=True
        self.boundary_band_fp+=result['boundary_band_false_positives']
        if 'latency' in f:self.latencies.append(f['latency'])
        # Bound memory while retaining exact running sums for averages.
        if len(self.errors)>100000:self.errors=self.errors[-100000:]
        if len(self.size_errors)>100000:self.size_errors=self.size_errors[-100000:]
        if len(self.latencies)>100000:self.latencies=self.latencies[-100000:]

    def boundary(self,msg,key):
        if msg.header.frame_id!='base_link':self.invalid+=1;return
        if any(b.side not in [1,2] or len(b.points)<2 or not 0<=b.confidence<=1 or not all(np.isfinite([p.x,p.y,p.z]).all() for p in b.points) for b in msg.boundaries):
            self.invalid+=1;return
        k=stamp_key(msg.header.stamp)
        now=seconds(self.get_clock().now().to_msg())
        if now-k/1e9>self.cfg['result_timeout']:return
        if key=='boundary_pred' and not self.boundary_active:
            self.boundary_active=True;self.boundary_start=k
        f=self.frames.setdefault(k,{})
        if key in f or f.get('boundary_done'):return
        f[key]=msg
        if key=='boundary_truth':self.process(k,f)
        if 'boundary_truth' in f and 'boundary_pred' in f:
            self.accumulate_boundary(score_boundary_frame(boundary_records(f['boundary_pred']),
                                                         boundary_records(f['boundary_truth']),self.cfg))
            f['boundary_done']=True

    def accumulate_boundary(self,result):
        self.boundary_points+=result['truth_points'];self.boundary_pred_points+=result['pred_points']
        self.boundary_covered+=result['covered'];self.boundary_correct+=result['correct']
        self.boundary_error+=result['error_sum'];self.boundary_frames+=1

    def flush(self):
        now=seconds(self.get_clock().now().to_msg())
        for k,f in list(self.frames.items()):
            if now-k/1e9<=self.cfg['result_timeout']:continue
            if self.active and k>=self.start and not f.get('done'):
                if all(key in f for key in ['truth','roi','boundary_truth']):
                    if 'pred' not in f:
                        f['pred']=Detection3DArray(header=f['truth'].header)
                        self.missing+=1
                    self.process(k,f)
                else:
                    self.incomplete_truth_frames+=1
            if self.boundary_active and k>=self.boundary_start and not f.get('boundary_done') and 'boundary_truth' in f:
                self.accumulate_boundary(score_boundary_frame([],boundary_records(f['boundary_truth']),self.cfg))
                self.boundary_missing+=1
            del self.frames[k]
        while len(self.frames)>512:self.frames.popitem(last=False)

    def report(self):
        result={'schema_version':2,'seed':self.seed,'status':'running' if self.active else 'waiting_for_algorithm',
                'tasks':['objects_3d','road_boundaries'],
                'scope':'LiDAR-return-visible objects, front ROI; not camera visibility or AP',
                'frames':self.count,'missing_frames':self.missing,'invalid_messages':self.invalid,'late_messages':self.late,
                'boundary_band_false_positives':self.boundary_band_fp,
                'incomplete_truth_frames':self.incomplete_truth_frames,
                'objectness':summarize(*map(int,self.objectness)),
                'semantic':summarize(*map(int,self.semantic)),
                'per_class':{k:summarize(v['tp'],v['fp'],v['fn']) for k,v in self.classes.items()},
                'mean_center_error_m':float(np.mean(self.errors)) if self.errors else None,
                'mean_size_absolute_error_m':float(np.mean(self.size_errors)) if self.size_errors else None,
                'mean_simulation_latency_s':float(np.mean(self.latencies)) if self.latencies else None,
                'error_window_max_samples':100000,'ignored_invisible_predictions':self.ignored,
                'boundary':{'status':'running' if self.boundary_active else 'waiting_for_algorithm',
                            **boundary_summary(self.boundary_points,self.boundary_pred_points,self.boundary_covered,self.boundary_correct),
                            'frames':self.boundary_frames,'missing_frames':self.boundary_missing,
                            'tolerance_m':self.cfg.get('boundary_tolerance',0.5),
                            'mean_error_on_covered_points_m':self.boundary_error/self.boundary_covered if self.boundary_covered else None}}
        data=json.dumps(result,indent=2,allow_nan=False)
        if rclpy.ok():
            self.pub.publish(String(data=data))
            marker=Marker()
            marker.header.frame_id='base_link';marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='score';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD
            marker.frame_locked=True;marker.pose.orientation.w=1.
            marker.pose.position.x=7.;marker.pose.position.z=5.
            marker.scale.z=0.55
            marker.color.r=1.;marker.color.g=1.;marker.color.b=1.;marker.color.a=1.
            marker.text=marker_text(result)
            self.score_pub.publish(marker)
        tmp=self.output/'summary.json.tmp';tmp.write_text(data);tmp.replace(self.output/'summary.json')


def main(args=None):
    rclpy.init(args=args);node=Evaluator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.report();node.destroy_node();rclpy.try_shutdown()

import copy

import pytest

from baja_perception_sim.evaluation import (score_frame, summarize,
                                            score_boundary_frame, boundary_summary)


CFG = {'confidence_threshold':0.25,'min_forward':2.,'max_distance':30.,
       'half_fov_deg':75.,'match_distance':1.,'boundary_tolerance':0.5,
       'boundary_sample_step':0.5,'boundary_obstacle_band':0.25}


def line(start=5.,end=15.,y=6.,side=1):
    return {'side':side,'confidence':1.,'points':[[start,y],[end,y]]}


def metrics(pred,gt):
    r=score_boundary_frame(pred,gt,CFG)
    return boundary_summary(r['truth_points'],r['pred_points'],r['covered'],r['correct'])


def test_false_positive_deduction_is_f1_not_an_extra_fixed_penalty():
    perfect=summarize(10,0,0)
    cone_fp=summarize(10,1,0)
    assert perfect['score_100']==100.
    assert cone_fp['score_100']==pytest.approx(2000/21)
    assert cone_fp['false_positive_penalty_100']==pytest.approx(100/21)
    assert cone_fp['score_100']+cone_fp['false_positive_penalty_100']==pytest.approx(100.)


def test_boundary_cone_cannot_match_a_nearby_real_target():
    target={'id':'target','class_id':'cone','position':[10.,5.4,0.15],'size':[0.3]*3,'score':1.}
    cone=copy.deepcopy(target);cone['position']=[10.,6.,0.15]
    r=score_frame([cone],[target],[target],CFG,[line()['points']])
    assert r['objectness']==(0,1,1)
    assert r['boundary_band_false_positives']==1
    r=score_frame([target],[target],[target],CFG,[line()['points']])
    assert r['objectness']==(1,0,0)


def test_exact_boundaries_score_full_without_any_cone_detections():
    gt=[line(),line(y=-6.,side=2)]
    m=metrics(copy.deepcopy(gt),gt)
    assert m['score_100']==100.


def test_missing_half_boundary_reduces_recall():
    m=metrics([line(end=10.)],[line()])
    assert m['precision']==1.
    assert m['recall']==pytest.approx(0.55)
    assert 0<m['score_100']<100.


def test_extra_wrong_boundary_reduces_precision():
    m=metrics([line(),line(y=3.)],[line()])
    assert m['recall']==1.
    assert m['precision']==0.5
    assert m['score_100']==pytest.approx(200/3)


def test_swapped_left_right_boundaries_are_wrong():
    m=metrics([line(side=2)],[line()])
    assert m['score_100']==0.


def test_point_density_and_outside_roi_do_not_improve_score():
    dense=line();dense['points']=[[5+i/10,6.] for i in range(101)]
    assert metrics([dense],[line()])==metrics([line()],[line()])
    gt=[line(start=2.,end=12.)]
    assert metrics([line(start=-100.,end=12.)],gt)['score_100']==100.


def test_empty_boundary_output_scores_zero_when_truth_exists():
    assert metrics([],[line()])['score_100']==0.

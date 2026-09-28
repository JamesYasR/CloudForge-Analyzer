#!/usr/bin/env bash
# 多凹坑验证: 依次跑 4 组, 全部使用同一份缓存轴线(与改造前一致)
cd /home/jamesyasr/dsh-workspace/CloudForge-Analyzer
D=analysis/cylinder_patch/out_multipit
B=./build/test_cyl_pothole
T=20
echo "##### 1) form 样本 W=90 T_local=0.35 (多凹坑主用例) #####"
taskset -c 0-15 $B PCDfiles/pothole_dent_form.pcd 1940 local=1 win=90 lthr=0.35 minpts=60 ctol=5 axis=$D/axis_form.txt > $D/v1_form_w90_t035.txt 2>&1
echo "##### 2) form 样本 W=90 T_local=0.50 (默认阈值) #####"
taskset -c 0-15 $B PCDfiles/pothole_dent_form.pcd 1940 local=1 win=90 lthr=0.50 minpts=60 ctol=5 axis=$D/axis_form.txt > $D/v2_form_w90_t050.txt 2>&1
echo "##### 3) form 样本 W=60 (窗口偏小对照) #####"
taskset -c 0-15 $B PCDfiles/pothole_dent_form.pcd 1940 local=1 win=60 lthr=0.35 minpts=60 ctol=5 axis=$D/axis_form.txt > $D/v3_form_w60.txt 2>&1
echo "##### 4) 理想单坑样本 pothole_test.pcd #####"
taskset -c 0-15 $B PCDfiles/pothole_test.pcd 1940 local=1 win=90 axis=$D/axis_test.txt > $D/v4_ptest.txt 2>&1
echo "##### 5) 真实无坑 1_cld.pcd #####"
taskset -c 0-15 $B "/media/jamesyasr/Shared/大创/点云/test2/1_cld.pcd" 1940 local=1 win=90 axis=$D/axis_1cld.txt > $D/v5_1cld.txt 2>&1
echo "##### 6) 口径A(local=0) 改造后 #####"
taskset -c 0-15 $B PCDfiles/pothole_dent_form.pcd 1940 local=0 axis=$D/axis_form.txt > $D/v6_local0.txt 2>&1
echo "##### 完成 #####"

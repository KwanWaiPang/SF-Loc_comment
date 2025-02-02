[comment]: <> 

<!-- PROJECT LOGO -->

<p align="center">

  <h1 align="center"> SEA-RAFT: Simple, Efficient, Accurate RAFT for Optical Flow
  </h1>

[comment]: <> (  <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://kwanwaipang.github.io/Blog_basedon_markdown/SF-Loc/">Blog</a> 
  | <a href="https://github.com/GREAT-WHU/SF-Loc">Original Github Page</a>
  </h3>
  <div align="justify">
  </div>

<br>

<!-- ~~~
rm -rf .git
git init
git add .
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:KwanWaiPang/SF-Loc_comment.git
git push -u origin main
~~~ -->

# 安装配置
~~~
# git clone --recurse-submodules https://github.com/GREAT-WHU/SF-Loc.git
# 注意原版的需要从submodules下载
git clone --recurse-submodules https://github.com/KwanWaiPang/SF-Loc_comment.git
cd SF-Loc

conda create -n sfloc python=3.10.11
conda activate sfloc
pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
pip install torch-scatter==2.0.9 -f https://data.pyg.org/whl/torch-1.11.0+cu113.html
pip install gdown tqdm numpy==1.25.0 numpy-quaternion==2022.4.3 opencv-python==4.7.0.72 scipy pyparsing matplotlib h5py 

# for A100 (CUDA12.2或cuda 12.1)
# conda remove --name sfloc --all 
pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu121
pip install torch-scatter -f https://data.pyg.org/whl/torch-2.1.0+cu121.html
pip install gdown tqdm numpy==1.25.0 numpy-quaternion==2022.4.3 opencv-python==4.7.0.72 scipy pyparsing matplotlib h5py 
pip install ninja

~~~

* 安装第三方库及GTSAM（注意要安装在conda环境下）
~~~
cd thirdparty

git clone https://github.com/yuxuanzhou97/gtsam.git
cd gtsam
mkdir build
cd build
cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.10.11
make python-install
~~~

* 安装sfloc
~~~
conda activate sfloc

python setup.py install
~~~

## 下载权重模型及数据集
* 权重模型用的就是droid的，此处直接用原本下载好的```/home/gwp/DBA-Fusion/droid.pth```
* 下载[WHU1023](https://whueducn-my.sharepoint.com/:u:/g/personal/2015301610143_whu_edu_cn/EQX_UOB79AhHlsSI7hb2Jd4B69qd367NCMHOAcFZi7N5Mg?e=gi9NP1)数据
* onedrive数据下载到服务器请见[博客](https://kwanwaipang.github.io/File/Blogs/Poster/ubuntu%E5%91%BD%E4%BB%A4%E8%A1%8C%E4%B8%8B%E8%BD%BD%E6%95%B0%E6%8D%AE.html#onedrive)

# 实验测试

## 运行mapping phase

1. 先运行下面代码（注意需要更改数据路径），实现多传感器DBA，作者说大概需要90分钟左右来跑完整个序列
~~~
conda activate sfloc

CUDA_VISIBLE_DEVICES=1 python launch_dba.py  # This would trigger demo_vio_WHU1023.py automatically.
~~~
* 此function应该就是相当于进行mapping的过程，会生成以下三个结果
    * poses_realtime.txt   IMU poses (both in world frame and ECEF frame) estimated by online multi-sensor DBA.
    * graph.pkl   Serialized GTSAM factors that store the multi-sensor DBA information.
    * depth_video.pkl   Dense depths estimated by DBA

2. 然后运行下面代码进行全局图优化
~~~
python sf-loc/post_optimization.py --graph results/graph.pkl --result_file results/poses_post.txt
~~~
* 会生成结果如下：
    * poses_post.txt   Estimated IMU poses after global optimization.

3. 接下来再通过下面代码来稀疏化关键帧地图
~~~
python sf-loc/sparsify_map.py --imagedir $DATASET/image_undist/cam0 --imagestamp $DATASET/stamp.txt --depth_video results/depth_video.pkl --poses_post results/poses_post.txt --calib calib/1023.txt --map_indices results/map_indices.pkl
~~~
* 会生成结果如下：
    * map_indices.pkl   Map frame indices (and timestamps), indicating a subset of all DBA keyframes.

4. 运行下面代码来生成lightweight structure frame map.同时通过[VPR-methods-evaluation](https://github.com/gmberton/VPR-methods-evaluation)中所提供的脚本可以很方便的使用不同的VRP方法。
~~~
python sf-loc/generate_sf_map.py --imagedir $DATASET/image_undist/cam0 --imagestamp $DATASET/stamp.txt --depth_video results/depth_video.pkl --poses_post results/poses_post.txt --calib calib/1023.txt --map_indices results/map_indices.pkl --map_file sf_map.pkl
~~~
* 生成最终的结果如下：
    * sf_map.pkl: The structure frame map, which is all you need for re-localization.

5. 大概50MB左右的轻量级地图文件可以获取。运行下面代码可验证全局pose估计的性能
~~~
python scripts/evaluate_map_poses.py
~~~

## 运行Localization phase 
* 使用[LightGlue](https://github.com/cvg/LightGlue)作为fine association，需要先配置安装
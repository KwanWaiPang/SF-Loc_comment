[comment]: <> 

<!-- PROJECT LOGO -->

<p align="center">

  <h1 align="center"> SEA-RAFT: Simple, Efficient, Accurate RAFT for Optical Flow
  </h1>

[comment]: <> (  <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://kwanwaipang.github.io/SF-Loc/">Blog</a> 
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
```bash
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
pip install einops
pip install scikit-learn
pip install open3d

```

* 安装第三方库及GTSAM（注意要安装在conda环境下）
```bash
cd thirdparty

git clone https://github.com/yuxuanzhou97/gtsam.git
cd gtsam
mkdir build
cd build
cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.10.11
make python-install
```

* 安装sfloc
```bash
conda activate sfloc

python setup.py install
```

## 下载权重模型及数据集
* 权重模型用的就是droid的，此处直接用原本下载好的```/home/gwp/DBA-Fusion/droid.pth```
* 下载[WHU1023](https://whueducn-my.sharepoint.com/:u:/g/personal/2015301610143_whu_edu_cn/EQX_UOB79AhHlsSI7hb2Jd4B69qd367NCMHOAcFZi7N5Mg?e=gi9NP1)数据
* onedrive数据下载到服务器请见[博客](https://kwanwaipang.github.io/File/Blogs/Poster/ubuntu%E5%91%BD%E4%BB%A4%E8%A1%8C%E4%B8%8B%E8%BD%BD%E6%95%B0%E6%8D%AE.html#onedrive)

# 实验测试

## 运行mapping phase

1. 先运行下面代码（注意需要更改数据路径），实现多传感器DBA。
```bash
conda activate sfloc

CUDA_VISIBLE_DEVICES=0 python launch_dba.py  # This would trigger demo_vio_WHU1023.py automatically (相当于写了个sh运行代码，包含了输入的参数~).
```
* 此function应该就是相当于进行mapping的过程，会生成以下三个结果
    * poses_realtime.txt   IMU poses (both in world frame and ECEF frame) estimated by online multi-sensor DBA.
    * graph.pkl   Serialized GTSAM factors that store the multi-sensor DBA information.
    * depth_video.pkl   Dense depths estimated by DBA
* 作者在github中提到，这步大概需要90分钟左右来跑完整个序列（但我在A100下测试却远不止一个半小时，都21个小时了，也可能是有其他任务在执行影响了速度吧~）。
<div align="center">
  <img src="./results/Figs/微信截图_20250203140629.png" width="60%" />
<figcaption>  
</figcaption>
</div>

* 此步应该就是进行定位、获取GTSAM的因子图以及DBA生成的Depth map

2. 然后运行下面代码进行全局图优化
```bash
python sf-loc/post_optimization.py --graph results/graph.pkl --result_file results/poses_post.txt
```
* 会生成结果如下：
    * poses_post.txt   Estimated IMU poses after global optimization.
<div align="center">
  <img src="./results/Figs/微信截图_20250203143125.png" width="60%" />
<figcaption>  
</figcaption>
</div>


3. 接下来再通过下面代码来稀疏化关键帧地图
```bash
python sf-loc/sparsify_map.py --imagedir WHU1023/image_undist/cam0 --imagestamp WHU1023/stamp.txt --depth_video results/depth_video.pkl --poses_post results/poses_post.txt --calib calib/1023.txt --map_indices results/map_indices.pkl
```
* 会生成结果如下：
    * map_indices.pkl   Map frame indices (and timestamps), indicating a subset of all DBA keyframes.
    * map_stamps.txt    应该是记录每个map的时间戳
* 获取的应该就是稀疏化后的地图（关键帧）
<div align="center">
  <img src="./results/Figs/微信截图_20250203150439.png" width="60%" />
<figcaption>  
</figcaption>
</div>

4. 运行下面代码来生成lightweight structure frame map.同时通过[VPR-methods-evaluation](https://github.com/gmberton/VPR-methods-evaluation)中所提供的脚本可以很方便的使用不同的VRP方法。
```bash
python sf-loc/generate_sf_map.py --imagedir WHU1023/image_undist/cam0 --imagestamp WHU1023/stamp.txt --depth_video results/depth_video.pkl --poses_post results/poses_post.txt --calib calib/1023.txt --map_indices results/map_indices.pkl --map_file sf_map.pkl
```
* 生成最终的结果如下：
    * sf_map.pkl: The structure frame map, which is all you need for re-localization.
* 其中会调用VPR-methods-evaluation来生成及存储识别的描述子
<div align="center">
  <img src="./results/Figs/微信截图_20250203161102.png" width="60%" />
<figcaption>  
</figcaption>
</div>

5. 最终大概50MB左右的轻量级地图文件可以获取（获取的为49.51MB）。运行下面代码可验证全局pose估计的性能
```bash
python scripts/evaluate_map_poses.py
```
<div align="center">
  <img src="./results/Figs/微信截图_20250203162845.png" width="60%" />
<figcaption> 
定位精度，实时的定位精度以及全局优化后的定位精度
</figcaption>
</div>


<div align="center">
  <img src="./mapping_error.svg" width="60%" />
<figcaption>  
</figcaption>
</div>

## 运行Localization phase 
* 使用[LightGlue(ICCV 2023)](https://github.com/cvg/LightGlue)作为fine association(进行特征点的匹配)，需要先配置安装
<div align="center">
  <table style="border: none; background-color: transparent;">
    <tr>
      <td style="width: 50%; border: none; padding: 0.01; background-color: transparent; vertical-align: middle;">
        <img src="https://github.com/cvg/LightGlue/raw/main/assets/easy_hard.jpg" width="100%" />
      </td>
      <td style="width: 50%; border: none; padding: 0.01; background-color: transparent; vertical-align: middle;">
        <img src="https://github.com/cvg/LightGlue/raw/main/assets/teaser.svg" width="100%" />
      </td>
    </tr>
  </table>
  <figcaption>
  </figcaption>
</div>

```bash
#下载下来并在当前环境下配置
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
rm -rf .git

conda activate sfloc
python -m pip install -e .
```
1. 下载[WHU0412](https://whueducn-my.sharepoint.com/:u:/g/personal/2015301610143_whu_edu_cn/EfyUSrS01jxFgFJFLmKlsuoBci59yljVbOm2A2LnVXi9dA?e=YJNxhv)数据集
2. 运行下面命名来验证定位性能(会导入前面生成的sf_map.pkl)
```bash
export DATASET_MAP=WHU1023
export DATASET_USER=WHU0412/WHU0412
python sf-loc/localization_sf_map.py --imagedir $DATASET_USER/image_undist/cam0 --map_file sf_map.pkl  --calib calib/0412.txt --map_extrinsic calib/1023.yaml --user_extrinsic calib/0412.yaml --user_odo_file $DATASET_USER/odo.txt --user_gt_file $DATASET_USER/gt.txt --map_gt_file $DATASET_MAP/gt.txt
```
* 注意原github作者给的命令有--enable_map_gt和--enable_user_gt，这两个都是要输入参数的，不是store的，因此去掉即可，因为默认就为true
* 而此命令应该就是调用map-based DBA会生成以下两个文件：
  * esult_coarse.txt   Coarse user localization results (position and map indice) based on VPR.
  * result_fine.txt   Fine user localization results (local and global poses).
3. 运行下面两个命令分别对两个精度进行验证
~~~
python scripts/evaluate_coarse_poses.py
python scripts/evaluate_fine_poses.py
~~~

<div align="center">
  <table style="border: none; background-color: transparent;">
    <tr>
      <td style="width: 50%; border: none; padding: 0.01; background-color: transparent; vertical-align: middle;">
        <img src="./coarse_error.svg" width="100%" />
      </td>
      <td style="width: 50%; border: none; padding: 0.01; background-color: transparent; vertical-align: middle;">
        <img src="./fine_error.svg" width="100%" />
      </td>
    </tr>
  </table>
  <figcaption>
  coarse pose vs fine pose 
  </figcaption>
</div>

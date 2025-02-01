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
conda create -n sfloc python=3.10.11
conda activate sfloc
pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 torchaudio==0.11.0 --extra-index-url https://download.pytorch.org/whl/cu113
pip install torch-scatter==2.0.9 -f https://data.pyg.org/whl/torch-1.11.0+cu113.html
pip install gdown tqdm numpy==1.25.0 numpy-quaternion==2022.4.3 opencv-python==4.7.0.72 scipy pyparsing matplotlib h5py 
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

# 下载权重模型及数据集
* 权重模型用的就是droid的，此处直接用原本下载好的```/home/gwp/DBA-Fusion/droid.pth```
* 下载[WHU1023](https://whueducn-my.sharepoint.com/:u:/g/personal/2015301610143_whu_edu_cn/EQX_UOB79AhHlsSI7hb2Jd4B69qd367NCMHOAcFZi7N5Mg?e=gi9NP1)数据

"""
============================================================================
grass_chopper パッケージ ビルド設定
============================================================================
colcon build 時にこのファイルが実行され、以下を行います:
  - Pythonモジュールのインストール
  - launch, urdf, worlds ディレクトリのコピー
  - 実行可能ファイル (エントリーポイント) の登録
============================================================================
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'grass_chopper'

setup(
    name=package_name,
    version='0.1.0',
    # Pythonパッケージを自動検出（test ディレクトリは除外）
    packages=find_packages(exclude=['test']),
    # ROS 2のament_indexに登録するデータファイル
    data_files=[
        # ament index にパッケージを登録（必須）
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml をshareディレクトリにコピー
        ('share/' + package_name, ['package.xml']),
        # launch ファイルをコピー
        # *launch.[pxy][yma]* は launch.py, launch.xml, launch.yaml にマッチ
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # URDF/Xacro ファイルをコピー
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        # World ファイルをコピー
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='you@example.com',
    description='草刈りロボット Gazebo Harmonicシミュレーション',
    license='Apache-2.0',
    tests_require=['pytest'],
    # エントリーポイント: ros2 run で実行できるコマンドを定義
    # コマンド名 = パッケージ名.モジュール名:関数名
    entry_points={
        'console_scripts': [
            'weeder_node = grass_chopper.weeder_node:main',
        ],
    },
)

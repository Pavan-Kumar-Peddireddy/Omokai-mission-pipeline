import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mission_bringup'


def collect_data_files(src_root, dest_prefix):
    """Recursively map every file under src_root into (dest_dir, [files])
    tuples, preserving the directory tree. Needed for models/ which has
    nested meshes/materials/textures subfolders that a flat glob() would
    silently drop -- Gazebo would then fail to resolve model:// URIs on
    a fresh install even though the source tree looks complete."""
    entries = []
    for dirpath, _dirnames, filenames in os.walk(src_root):
        if not filenames:
            continue
        rel_dir = os.path.relpath(dirpath, src_root)
        dest_dir = os.path.join('share', package_name, dest_prefix) if rel_dir == '.' \
            else os.path.join('share', package_name, dest_prefix, rel_dir)
        entries.append((dest_dir, [os.path.join(dirpath, f) for f in filenames]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files - Explicitly structured path join for glob compliance
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

        # Config files (YAMLs + RViz configs -- *.rviz was previously
        # missing from this glob, so nav2_default_view.rviz was NEVER
        # actually installed/copied by colcon on any prior build, no
        # matter what its source content was.)
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz'))),

        # URDF models (SDF, URDF, etc.)
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),

        # Map files (.yaml and .pgm)
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),

        # World files (.sdf or .world)
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
    ] + collect_data_files('models', 'models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='todo@todo.com',
    description='Bringup package for LLM and Executor pipelines',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf = mission_bringup.odom_to_tf:main',
        ],
    },
)

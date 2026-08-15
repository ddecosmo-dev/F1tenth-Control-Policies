from setuptools import setup
import os

package_name = 'mapless_lstm_racer'

# Collect data files (models/) so they are installed into share/<package>/models
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

models_root = os.path.join(os.path.dirname(__file__), 'models')
if os.path.isdir(models_root):
    for root, dirs, files in os.walk(models_root):
        if not files:
            continue
        # target directory under share/<package>/models/... preserving subdirs
        rel_path = os.path.relpath(root, models_root)
        if rel_path == '.':
            target_dir = os.path.join('share', package_name, 'models')
        else:
            target_dir = os.path.join('share', package_name, 'models', rel_path)
        filepaths = [os.path.join(root, f) for f in files]
        data_files.append((target_dir, filepaths))

# include config/ and launch/ directories if present
for extra in ('config', 'launch'):
    extra_root = os.path.join(os.path.dirname(__file__), extra)
    if os.path.isdir(extra_root):
        for root, dirs, files in os.walk(extra_root):
            if not files:
                continue
            rel_path = os.path.relpath(root, extra_root)
            if rel_path == '.':
                target_dir = os.path.join('share', package_name, extra)
            else:
                target_dir = os.path.join('share', package_name, extra, rel_path)
            filepaths = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, filepaths))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'onnxruntime', 'joblib', 'numpy<=1.22.0', 'scikit-learn', 'PyYAML'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='reggiedecossard@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lstm_drive = mapless_lstm_racer.lstm_drive_node:main',
            'lstm_logger = mapless_lstm_racer.lstm_logger_node:main',
        ],
    },
)

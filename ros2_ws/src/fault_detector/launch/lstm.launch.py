import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    venv_python = '/home/faisal-mazloum/Desktop/Machine_Learning/ml_venv/bin/python3'
    
    env = {
        **os.environ,
        'PYTHONPATH': (
            '/home/faisal-mazloum/remora_ws/install/fault_detector/lib/python3.12/site-packages:'
            '/home/faisal-mazloum/Desktop/Machine_Learning/ml_venv/lib/python3.12/site-packages:'
            + os.environ.get('PYTHONPATH', '')
        ),
        'TF_CPP_MIN_LOG_LEVEL': '3',
    }
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[venv_python, '-m', 'fault_detector.lstm_fault_detector'],
            env=env,
            output='screen'
        )
    ])
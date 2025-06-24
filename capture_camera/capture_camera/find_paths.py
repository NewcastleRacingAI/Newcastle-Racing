import os

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))

    print(current_dir)

    # Define the relative path to the artifacts directory
    artifacts_dir = os.path.join(current_dir, '..', 'artifacts')
    
    # Paths to your specific files
    weights_path = os.path.join(artifacts_dir, 'pretrained_yolo.weights')
    cfg_path = os.path.join(artifacts_dir, 'yolo_baseline.cfg')
    
    # Print the paths to verify they are correct
    print("Weights Path:", weights_path)
    print("Config Path:", cfg_path)

if __name__ == '__main__':
    main()
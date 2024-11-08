#check if mnist_model.pth exists
import os
import torch
import cv2
import torchvision
import matplotlib.pyplot as plt
if not os.path.exists('mnist_model.pth'):
    import requests
    # Download the model weights
    response = requests.get('https://drive.google.com/uc?export=download&id=1RFq0KQ0npXcvz0XiGKYqvybT71lyofrw')
    with open('mnist_model.pth', 'wb') as f:
        f.write(response.content)

    print("mnist_model.pth downloaded successfully")

normalize = torchvision.transforms.Compose([
      torchvision.transforms.ToPILImage(),
      torchvision.transforms.Resize((28, 28)),
      torchvision.transforms.ToTensor(),
      torchvision.transforms.Lambda(lambda x: (x - 0.5) * 2)
  ])

model = torch.jit.load('mnist_model.pth', map_location=torch.device('cpu'))
model.eval()

def process(image):
    #TODO: return np.array of same size as image
    pass

def recognize_digit():
    cap = cv2.VideoCapture(0)
    last_prediction = None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Display the last prediction if available
        if last_prediction is not None:
            cv2.putText(frame, f"Predicted: {last_prediction}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Camera Feed', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # Space key
            # Process the image
            processed_image = process(frame)
            processed_image = normalize(processed_image).unsqueeze(0).float()
            # Pass through the model
            with torch.no_grad():
                output = model(processed_image)
                last_prediction = output.argmax().item()
        
        elif key == ord('c'):  # 'c' key to clear prediction
            last_prediction = None
        
        elif key == ord('q'):  # 'q' key to quit
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    recognize_digit()
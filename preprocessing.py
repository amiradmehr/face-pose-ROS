from helper import *

def map01(nparray: np.array):
    nparray.dtype=float
    array_shape = nparray.shape
    for i in range(array_shape[1]):
        nparray[:,i] = (nparray[:,i] - min(nparray[:,i])) / (max(nparray[:,i]) - min(nparray[:,i]))

def centering_face(points):
    err = points[1] - np.ones((3,)) * .5
    centered = points - err
    return centered

def preprocess_mesh():
    pass
def facemesh_centered(image):
    
    width  = image.shape[1]
    height = image.shape[0]
    LANDMARKS = []
    
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(static_image_mode=True,
                                    max_num_faces=1,
                                    min_detection_confidence=0.5, min_tracking_confidence=0.1)
    
    
    # processing the image
    results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    if results.multi_face_landmarks:
        # Draw face landmarks of each face.
        for face_landmarks in results.multi_face_landmarks: 
            # saving lanmarks in another list
            LANDMARKS.append(face_landmarks.landmark)

            # extracting lanmark coordinates
            # points = extract_landmarks(LANDMARKS, width, height)
            points_normal = extract_landmarks(LANDMARKS, width, height, normal=True)
            centered_mesh = centering_face(points_normal)
            map01(centered_mesh)
            # centered_mesh[:,0] = (centered_mesh[:,0] - min(centered_mesh[:,0])) / (max(centered_mesh[:,0]) - min(centered_mesh[:,0]))

    else:
        print("some parts of face is missing")

    return centered_mesh

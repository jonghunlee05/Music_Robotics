import cv2
import mediapipe as mp
import pygame
import numpy as np

pygame.mixer.init()

notes = {
    'C': pygame.mixer.Sound("C.wav"),
    'D': pygame.mixer.Sound("D.wav"),
    'E': pygame.mixer.Sound("E.wav"),
    'F': pygame.mixer.Sound("F.wav"),
    'G': pygame.mixer.Sound("G.wav"),
    'A': pygame.mixer.Sound("A.wav"),
    'B': pygame.mixer.Sound("B.wav"),
    'C1': pygame.mixer.Sound("C1.wav")
}

finger_states = {

    'C' : False,
    'D' : False,
    'E' : False,
    'F' : False,
    'G' : False,
    'A' : False,
    'B' : False,
    'C1' : False
}

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands = 2)
mp_drawing = mp.solutions.drawing_utils

def play_note_based_on_fingers(hand_landmarks, hand_index):
    global finger_states
    if hand_index == 0:  # Left hand
        if hand_landmarks.landmark[8].y > hand_landmarks.landmark[7].y:
            if not finger_states['C']:
                notes['C'].play()
                finger_states['C'] = True
        else:
            finger_states['C'] = False

        if hand_landmarks.landmark[12].y > hand_landmarks.landmark[11].y:
            if not finger_states['D']:
                notes['D'].play()
                finger_states['D'] = True
        else:
            finger_states['D'] = False


        if hand_landmarks.landmark[16].y > hand_landmarks.landmark[15].y:
            if not finger_states['E']:
                notes['E'].play()
                finger_states['E'] = True
        else:
            finger_states['E'] = False


        if hand_landmarks.landmark[20].y > hand_landmarks.landmark[19].y:
            if not finger_states['F']:
                notes['F'].play()
                finger_states['F'] = True
        else:
            finger_states['F'] = False


    elif hand_index == 1:
        if hand_landmarks.landmark[20].y > hand_landmarks.landmark[19].y:
            if not finger_states['G']:
                notes['G'].play()
                finger_states['G'] = True

        else:
            finger_states['G']= False


        if hand_landmarks.landmark[16].y > hand_landmarks.landmark[15].y:
            if not finger_states['A']:
                notes['A'].play()
                finger_states['A'] = True
        else:
            finger_states['A'] = False


        if hand_landmarks.landmark[12].y > hand_landmarks.landmark[11].y:
            if not finger_states['B']:
                notes['B'].play()
                finger_states['B'] = True
        else:
            finger_states['B'] = False

        
        if hand_landmarks.landmark[8].y > hand_landmarks.landmark[7].y:
            if not finger_states['C']:
                notes['C1'].play()
                finger_states['C1'] = True
        else:
            finger_states['C1'] = False

cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, img = cap.read()
    if not success:
        print("Ignoring empty camera fram.")
        continue

    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    if results.multi_hand_landmarks:
        for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            play_note_based_on_fingers(hand_landmarks, i)

    cv2.imshow('Hand Gesture Music Player', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    key = cv2.waitKey(5) & 0xFF
    if key == ord('q') or key == 27:  # 'q' key or ESC key
        break

cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np
import time
import pyautogui


def transition_colors(sequence):

    img = np.zeros((500, 500, 3), dtype=np.uint8)
    img[:] = (0, 0, 255)  # Tela vermelha
    cv2.imshow("Transmitter", img)
    cv2.waitKey(1)  # Atualiza a janela e espera 1ms para exibir corretamente
    time.sleep(3)  # Espera 3 segundos


    for char in sequence:
        white = (255, 255, 255)
        black = (0, 0, 0)

        if char == "0":
            color = white
        elif char == "1":
            color = black

        img = np.zeros((500, 500, 3), dtype=np.uint8)
        img[:] = color

        cv2.imshow("Transmitter", img)
        cv2.waitKey(1)  



        time.sleep(3)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return False
    
    
    img = np.zeros((500, 500, 3), dtype=np.uint8)
    img[:] = (255, 0 , 0)  # Tela Azul
    cv2.imshow("Transmitter", img)
    cv2.waitKey(1)  # Atualiza a janela e espera 1ms para exibir corretamente
    time.sleep(3)  # Espera 3 segundos

    

    return True


def centralize_window(window_name, width, height):
    screen_width, screen_height = pyautogui.size()

    x = (screen_width - width) // 2
    y = (screen_height - height) // 2

    cv2.moveWindow(window_name, x, y)


def run_transmitter(sequence):
    width = 1200
    height = 1080

    cv2.namedWindow("Transmitter", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Transmitter", width, height)
    centralize_window("Transmitter", width, height)

    continue_program = True
    while continue_program:
        continue_program = transition_colors(sequence)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    transmitter_message = "0101"
    run_transmitter(transmitter_message)
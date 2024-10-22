import cv2
import numpy as np
import time 

def recognize_bits(bit_region):
    # Converter a região do bit para escala de cinza
    gray_region = cv2.cvtColor(bit_region, cv2.COLOR_BGR2GRAY)

    #Aplicar filtro 
    
    # Calcular a intensidade média na região
    mean_intensity = np.mean(gray_region)
    
    # Definir um limiar para diferenciar preto (0) e branco (1)
    threshold = 127  # Típico valor para diferenciar preto e branco
    
    if mean_intensity > threshold:
        return 0  # Branco
    else:
        return 1  # Preto
    

    
def is_synchronized(bit_region):
    hsv_image = cv2.cvtColor(bit_region, cv2.COLOR_BGR2HSV)

    #Definir os limites da cor vermelha no espaço de cor HSV
    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])

    #Máscara para detectar a cor vermelha
    mask = cv2.inRange(hsv_image,lower_red,upper_red)

    red_pixels = cv2.countNonZero(mask)
    return red_pixels > (bit_region.size//4)

def is_blue(bit_region):
    hsv_image = cv2.cvtColor(bit_region, cv2.COLOR_BGR2HSV)

    #Definir os limites da cor azul no espaço de cor HSV
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([130,255,255])

    #Máscara para detectar a cor vermelha
    mask = cv2.inRange(hsv_image,lower_blue,upper_blue)

    blue_pixels = cv2.countNonZero(mask)
    return blue_pixels > (bit_region.size//4)


def receive_bits():

    # Inicializar a webcam
    cap = cv2.VideoCapture(0)

    # cap.set(cv2.CAP_PROP_FPS, 30)  # Ajustar a taxa de quadros para 30 fps
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Definir a resolução para 640x480
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    bit_sequence =[]
    synchronized=False

    print("Aguardando sincronização")



    #Loop para detectar a tela vermelha.
    while True:
        # Capturar o frame da webcam. ret é um valor booleano.
        ret, frame = cap.read()

        if not ret:
            print("Falha na captura da webcam.")
            break

        #Região onde os bits estão localizados. Área específica para análise (x, y, largura, altura)
        bit_region = frame[100:200, 100:200]

        # Mostrar a área do bit
        cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), 2)

        cv2.imshow("Sincronização", frame)

        if not synchronized:
            if is_synchronized(bit_region):
                print("Sincronização detectada!!")
                synchronized = True
                print("Sincronização concluída!!")
              
        
        if synchronized:                
            # Reconhecer o bit
            bit_value = recognize_bits(bit_region)
            
            # Exibir o valor reconhecido
            cv2.putText(frame, f'Bit: {bit_value}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
                
            bit_sequence.append(bit_value)
            
            # Exibir o frame com o resultado
            # cv2.imshow("Reconhecimento de Bits", frame)
            
            # Verificar se a cor azul está presente (encerrar transmissão)
            if is_blue(bit_region):
                print("Cor azul detectada! Encerrando transmissão.")
                bit_sequence = []  # Reinicia a sequência de bits
                synchronized = False  # Volta a esperar pela sincronização vermelhaq
                print("Aguardando nova sincronização com a tela vermelha...")
            
            
            time.sleep(4)
        # Parar com a tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        print (bit_sequence)

    # Liberar os recursoss
    cap.release()

    cv2.destroyAllWindows()
receive_bits()
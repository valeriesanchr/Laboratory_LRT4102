#Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar
#un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe
#ṕroporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while
#debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos
#intentos el usuario logró adivinar el número

# importamos la librería random para usar numeros aleatorios
import random

# Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)
intentos = 0

print("¡Adivina el número secreto entre 1 y 10!")

while True:
    try:
        # Solicitar un número al usuario
        numero_usuario = int(input("Ingresa tu número: "))
        intentos += 1
        
        # Comprobar si el número es correcto
        if numero_usuario < numero_secreto:
            print("Demasiado bajo. Intenta de nuevo.")
        elif numero_usuario > numero_secreto:
            print("Demasiado alto. Intenta de nuevo.")
        else:
            print(f"¡Felicidades! Adivinaste el número en {intentos} intentos.")
            break
    except ValueError:
        print("Por favor, ingresa un número válido.")

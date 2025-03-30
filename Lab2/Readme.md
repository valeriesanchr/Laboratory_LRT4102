# Laboratory 2 Report
For this lab exercise, we created several programs that tested our knowledge in different areas.

## Lab2 Basic

![image](https://github.com/user-attachments/assets/ca0eb402-e3cd-4bf3-b4f9-ab9dc60e1749)

```python
x = 5
y = "John"
print(x)
print(y)
```
## Estructuras en python
### Estructura de for en Python
El bucle for se usa para iterar sobre secuencias (listas, tuplas, cadenas, rangos, etc.).
```python
for num in range(3):  # Itera desde 0 hasta 2
    print("Número:", num)
```
### Estructura de while en Python
El bucle while se ejecuta mientras una condición sea True.
```python
contador = 0
while contador < 3:  # Se ejecuta mientras contador sea menor que 3
    print("Contador:", contador)
    contador += 1
```
# Python orientado a objetos

La programación orientada a objetos (OOP) es un paradigma de programación que permite estructurar programas agrupando propiedades y comportamientos dentro de objetos individuales.

Por ejemplo, un objeto puede representar a una persona con características como nombre, edad y dirección, junto con acciones como caminar, hablar, respirar y correr. También podría representar un correo electrónico, con propiedades como destinatarios, asunto y contenido, y funciones como agregar archivos adjuntos y enviarlo.

En otras palabras, la programación orientada a objetos es una forma de modelar elementos del mundo real, como automóviles, así como relaciones entre entidades, como empresas y empleados o estudiantes y profesores. A través de OOP, se representan estos elementos como objetos de software que contienen datos y pueden realizar ciertas operaciones.

* **Encapsulación** permite agrupar datos (atributos) y comportamientos (métodos) dentro de una clase para crear una unidad cohesionada. Al definir métodos para controlar el acceso y la modificación de los atributos, la encapsulación ayuda a mantener la integridad de los datos y promueve un código modular y seguro.
* **Herencia** permite la creación de relaciones jerárquicas entre clases, permitiendo que una subclase herede atributos y métodos de una clase padre. Esto promueve la reutilización del código y reduce la duplicación.
* **Abstracción** se centra en ocultar los detalles de implementación y exponer solo la funcionalidad esencial de un objeto. Al imponer una interfaz consistente, la abstracción simplifica la interacción con los objetos, permitiendo a los desarrolladores centrarse en qué hace un objeto en lugar de cómo lo hace.
* **Polimorfismo** permite tratar objetos de diferentes tipos como instancias de un mismo tipo base, siempre que implementen una interfaz o comportamiento común. El duck typing de Python lo hace especialmente adecuado para el polimorfismo, ya que permite acceder a atributos y métodos de los objetos sin necesidad de preocuparse por su clase real.

# Solución a problemas

## Problema 1
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre
en pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enteros
positivos puede ser calculada de la siguiente forma: 

Solución:
```python
# le pedimos al usuario que ingrese un número
n = int(input("Enter your desired n number."))

# formula para calcular la suma de los n primeros números
suma = n*(n+1)/2

#imprimimos el resultado
print(f"The sum of the numbers is {suma}")
```
El código de este problema fue bastante simple, ya que solo se necesitaba almacenar el número indicado en una variable, y utilizar la fórmula, para posteriormente imprimirlo.

## Problema 2
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora.
Después debe mostrar por pantalla la paga que le corresponde.

Solución:
```python
#le decimos al usuario que nos dé los datos necesarios
n_horas = int(input("¿Cuál es el número de horas trabajadas? "))
n_paga = int(input("¿Cuánto te pagan por hora? "))

#calculamos el pago total
pago_total = n_paga*n_horas

#imprimimos el resultado
print(f"La paga que te corresponde es de ${pago_total}")
```
Para elaborar esta solución, se hizo uso uso de comandos simples. Lo único que se hace es pedirle al usuario los datos necesarios, y después hacer una multiplicación para poder calcular el pago total. Posteriormente, se imprime el resultado.

## Problema 3
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores.
Imprime el nombre y el sueldo a pagar de cada operador.

Solución:
```python
#escribimos los datos necesarios para el problema
lista_nombres = ["Pepito", "Pedrito", "Juanito", "Carlitos", "Fulanito","Menguanito"]
lista_sueldo_hora= [50,76,10,14,67,55]
lista_horas_trabajadas = [4,8,9,5,1,7]

#imprimimos el resultado con un ciclo for que iterará todos los nombres y multiplicará 
# las horas trabajadas por el sueldo por hora
for i in range(6):
    
    print(str(lista_nombres[i]) + " will be paid " + str(lista_horas_trabajadas[i]*lista_sueldo_hora[i])+ "$")
```
Para esta solución, hicimos del uso de vectores y un ciclo for. Los vectores nos permitieron almacenar los distintos tipos de datos necesarios, y después con un ciclo for se iteró todas las listas para poder imprimir los datos. En este ciclo for, se hizo la multiplicación de la posición i de las lista de suelo por hora, por la lista de horas trabajadas. 
## Problema 4

Crea una lista llamada numeros que contenga al menos 10 números. Calcula el promedio de los números pares y el producto de los números impares. Imprime los resultados.
```python
## Crea una lista llamada numeros que contenga al menos 10 números.
## Calcula el promedio de los números pares y el producto de los números impares.
## Imprime los resultados

lista_numeros = [1,2,3,4,5,6,7,8,9,10]

suma_pares = 0
producto_impares = 1

for i in range(10):
    if lista_numeros[i] % 2 == 0:
        suma_pares += lista_numeros[i]
    else:
        producto_impares *= lista_numeros[i]
        
promedio = float(suma_pares) / 5  # There are 5 even numbers in the list
print("Promedio de números pares:", promedio)
print("Producto de números impares:", producto_impares)
```
Se hace un ciclo for de 10 elementos, para poder considerar los 10 números que existen en el vector. En este ciclo for se tiene un if, el cual va a checar si el número es par o impar. Dependiendo de esto, se va a realizar la acción necesaria (sacar el promedio o el producto). 
## Problema 5
```python
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

```
En este programa fue necesario importar la librería random para pdoer hacer uso de la función random.randomint(). Esta función nos permite generar un número pseudo-random. Una vez que este número fue generado, el programa entra en un ciclo while. Aquí tenemos un if, el cual checa si el número ingresado es menor que el número secreto. 

En caso de no serlo, pasamos el siguiente caso, un elif, el cual pregunta si el número es más alto que el número secerto. El programa no termina hasta que el usuario cae en el caso "else", lo cual significa que ha adivinado el número. Este caso contiene el comando "break", que le indica al programa que debe romper el ciclo while. 

En caso de ingresar un dato no válido, tenemos el caso "except" el cual nos permite atrapar anomalías sin romper el ciclo while. Este except le pide a los usuarios ingresar un dato válido.

## Problema 6
```python
import random

# definir tamaño del mapa
tamaño = 5
# Crear la matriz del terreno con espacios libres representados por "o"
terreno = [['o' for _ in range(tamaño)] for _ in range(tamaño)]
# Definir posición inicial y destino
pos_actual = (0, 0)
pos_final = (tamaño - 1, tamaño - 1)

# Almacenar la ruta recorrida
trayectoria = []

# Direcciones de movimiento disponibles
movs = ['D', 'B', 'I', 'A']
direc = 'D'
```
En esta parte hacemos el set up inicial del código. Indicamos el tamaño del mapa, así como crear el terreno que por el momento estará libre de obstáculos. Sabemos que la posición inicial es (0,0) y que la posición final siempre será (tamaño-1, tamaño-1). También creamos un vector vacío que posteriormente almacenará la ruta que habrá recorrido el robot. Tenemos cuatro direcciones de movimiento posibles: izquierda, derecha, arriba y abajo, representadas por D, B, I , A  El robot inicia moviéndose hacia la derecha.

```python
# Generar obstáculos aleatorios en el mapa sin bloquear el inicio ni la meta
num_obstaculos = random.randint(tamaño, tamaño * 2)
for _ in range(num_obstaculos):
    while True:
        fila, col = random.randint(0, tamaño - 1), random.randint(0, tamaño - 1)
        if (fila, col) not in [(0, 0), pos_final]:  # Evitar bloquear la entrada y salida
            terreno[fila][col] ='X'
            break
```
Se añaden los obstáculos al terreno. Se tiene cuidado para que no se creen obstáculos ni en la salida ni en la meta.

```python
# Añadir posición inicial al camino recorrido
trayectoria.append(pos_actual)

# Diccionario de desplazamientos
desplazamiento = {
    'D': (0, 1),   # Derecha
    'I': (0, -1),  # Izquierda
    'A': (-1, 0),  # Arriba
    'B': (1, 0)    # Abajo
}
```
Al vector trayectoria se le añade la posición actual cada que el robot hace un movimiento nuevo. También se explica cómo se puede mover el robot. 
```python
# Función anónima para verificar si un movimiento es válido
es_valido = lambda x, y: 0 <= x < tamaño and 0 <= y < tamaño and terreno[x][y] != 'X'

# Iniciar movimiento del robot
fila, col = pos_actual
while (fila, col) != pos_final:
    mov_exitoso = False
    dx, dy = desplazamiento[direc]
    nueva_fila, nueva_col = fila + dx, col + dy

    if es_valido(nueva_fila, nueva_col):
        fila, col = nueva_fila, nueva_col
        trayectoria.append((fila, col))
        mov_exitoso = True
    else:
        direc = movs[(movs.index(direc) + 1) % 4]  # Cambia de dirección en sentido horario

    if not mov_exitoso and not any(es_valido(fila + dx, col + dy) for dx, dy in desplazamiento.values()):
        print("\nNo hay ruta disponible.")
        break
```
Se inicia el movimiento del robot, y se genera un while que se mantendrá activo hasta que el robot llegue a la posición final. Mientras tanto, tenemos los comandos que le permitirán al robot seguirse moviendo en el mapa.

```python
# Mostrar resultados
if (fila, col) == pos_final:
    print("\nEl robot llegó al destino.")

# Imprimir el mapa del terreno con la trayectoria recorrida
print("\nMapa del Terreno:")
for i in range(tamaño):
    for j in range(tamaño):
        if (i, j) in trayectoria:
            print("", end=" ")  # Marca el camino con ""
        else:
            print(terreno[i][j], end=" ")
    print()
```
Una vez que el robot llegó al destino, se imprime un mensaje indicando esto. Además, se imprime el mapa del terrreno.
```python
# Crear una nueva matriz para visualizar la trayectoria con flechas
mapa_flechas = [['o' for _ in range(tamaño)] for _ in range(tamaño)]
indicadores = {(0, 1): '→', (0, -1): '←', (-1, 0): '↑', (1, 0): '↓'}

for k in range(len(trayectoria) - 1):
    f1, c1 = trayectoria[k]
    f2, c2 = trayectoria[k + 1]
    dx, dy = f2 - f1, c2 - c1
    mapa_flechas[f1][c1] = indicadores[(dx, dy)]

mapa_flechas[pos_final[0]][pos_final[1]] = 'F'  # Marca la meta con la letra F

print("\nMapa con la Ruta Seguida:")
for fila in mapa_flechas:
    print(" ".join(fila))
```
Finalmente, se crea la matriz que será capaz de mostrar la trayectoria que siguió el robot con una serie de flechas. El for nos ayuda a llenar esta matriz con la ruta seguida. Finalmente, se marca la meta con la letra F, y se imprime el mapa. 

## Problema 7
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema
en Python que permita:
* Crear productos, cada uno con un nombre, precio y cantidad en stock.
* Actualizar la cantidad en stock cuando se venden productos.
* Mostrar la información de un producto con su disponibilidad.
* Calcular el valor total del inventario (precio × cantidad de cada producto).
```python
class Producto:
    def __init__(self, nombre, precio, cantidad):
        # Inicializa los atributos del producto
        self.nombre = nombre
        self.precio = precio
        self.cantidad = cantidad
```
Aquí inicializamos nuestro código, creando una clase llamada Producto. También definimos una función que nos permite inicializar los atributos del producto, generando tres atributos: nombre, precio, y cantidad.

```python    
    def vender(self, cantidad_vendida):
        # Reduce la cantidad en stock si hay suficiente disponibilidad
        if cantidad_vendida <= self.cantidad:
            self.cantidad -= cantidad_vendida
            print(f"Se vendieron {cantidad_vendida} unidades de {self.nombre}.")
        else:
            print("No hay suficiente stock disponible.")
```
Ahora creamos la función vender, en donde definimos el comportamiento cuando se intenta realizar una venta. Primero checa si hay suficiente stock, en caso de haberlo, procede a reducir el número de stock. 
```python
    def mostrar_info(self):
        # Muestra la información del producto y su disponibilidad
        disponibilidad = "Disponible" if self.cantidad > 0 else "Agotado"
        print(f"Producto: {self.nombre}, Precio: ${self.precio}, Stock: {self.cantidad}, Estado: {disponibilidad}")
```
Esta función nos sirve para mostrar la informacipon del producto. Indica el nombre, el precio, si hay stock, y si este producto está disponible para su venta.

```python    
    def valor_total(self):
        # Calcula el valor total del inventario del producto
        return self.precio * self.cantidad
```
Una simple función que realiza una multiplicación de precio por cantidad para saber el valor total de lo que hay en stock de cierto producto.
```python
# Lista para almacenar los productos
productos = []
```
Se genera un vector vacío en donde posteriormente se almacenan los productos que ingrese el usuario.

```python
def agregar_producto():
    nombre = input("Ingrese el nombre del producto: ")
    precio = float(input("Ingrese el precio del producto: "))
    cantidad = int(input("Ingrese la cantidad en stock: "))
    productos.append(Producto(nombre, precio, cantidad))
    print("Producto agregado exitosamente.")
```
Es la función que se ocupa cuando el usuario desea agregar un producto. Se necesita ingresar el nombre, el precio, y la cantidad de producto. Estos datos se agregan al vector producto, exitosamente agregándolos a stock.
```python
def vender_producto():
    nombre = input("Ingrese el nombre del producto a vender: ")
    for producto in productos:
        if producto.nombre == nombre:
            cantidad = int(input("Ingrese la cantidad a vender: "))
            producto.vender(cantidad)
            return
    print("Producto no encontrado.")
```
Cuando se realizar la venta de un producto, se llama esta función. Primero busca que el producto sí exista en nuestro vector que almacena los productos, y después llama a la función vender, ingresando el dato de cantidad para que realice la operación vender con ese dato.
```python
def mostrar_productos():
    if not productos:
        print("No hay productos registrados.")
        return
    for producto in productos:
        producto.mostrar_info()
        print(f"Valor total del inventario: ${producto.valor_total()}")
```
Esta función permite mostrar todo el stock disponible, así como dar el valor de todo lo que hay en stock.
```python
def menu():
    while True:
        print("\n--- Menú de Inventario ---")
        print("1. Agregar producto")
        print("2. Vender producto")
        print("3. Mostrar productos")
        print("4. Cerrar")
        opcion = input("Seleccione una opción: ")
        
        if opcion == "1":
            agregar_producto()
        elif opcion == "2":
            vender_producto()
        elif opcion == "3":
            mostrar_productos()
        elif opcion == "4":
            print("Cerrando el sistema...")
            break
        else:
            print("Opción no válida. Intente nuevamente.")
```
Esta es la definición de la función menú que se le muestra al usuario, el cual va a llamar a varias de las funciones antes definidas para poder realizar dicha operación. 
```python
# Ejecutar el menú
menu()
```
Ejecuta el menú que se le muestra al usuario. 

# Referencias

1. W3Schools.com. (n.d.). https://www.w3schools.com/python/python_variables.asp
2. Amos, D. (2024, December 15). Object-Oriented Programming (OOP) in Python. https://realpython.com/python3-object-oriented-programming/#what-is-object-oriented-programming-in-python
3. ¿Qué es Python? - Explicación del lenguaje Python - AWS. (n.d.). Amazon Web Services, Inc. https://aws.amazon.com/es/what-is/python/










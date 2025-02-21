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
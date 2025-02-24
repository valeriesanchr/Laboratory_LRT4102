class Producto:
    def __init__(self, nombre, precio, cantidad):
        # Inicializa los atributos del producto
        self.nombre = nombre
        self.precio = precio
        self.cantidad = cantidad
    
    def vender(self, cantidad_vendida):
        # Reduce la cantidad en stock si hay suficiente disponibilidad
        if cantidad_vendida <= self.cantidad:
            self.cantidad -= cantidad_vendida
            print(f"Se vendieron {cantidad_vendida} unidades de {self.nombre}.")
        else:
            print("No hay suficiente stock disponible.")
    
    def mostrar_info(self):
        # Muestra la información del producto y su disponibilidad
        disponibilidad = "Disponible" if self.cantidad > 0 else "Agotado"
        print(f"Producto: {self.nombre}, Precio: ${self.precio}, Stock: {self.cantidad}, Estado: {disponibilidad}")
    
    def valor_total(self):
        # Calcula el valor total del inventario del producto
        return self.precio * self.cantidad

# Lista para almacenar los productos
productos = []

def agregar_producto():
    nombre = input("Ingrese el nombre del producto: ")
    precio = float(input("Ingrese el precio del producto: "))
    cantidad = int(input("Ingrese la cantidad en stock: "))
    productos.append(Producto(nombre, precio, cantidad))
    print("Producto agregado exitosamente.")

def vender_producto():
    nombre = input("Ingrese el nombre del producto a vender: ")
    for producto in productos:
        if producto.nombre == nombre:
            cantidad = int(input("Ingrese la cantidad a vender: "))
            producto.vender(cantidad)
            return
    print("Producto no encontrado.")

def mostrar_productos():
    if not productos:
        print("No hay productos registrados.")
        return
    for producto in productos:
        producto.mostrar_info()
        print(f"Valor total del inventario: ${producto.valor_total()}")

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

# Ejecutar el menú
menu()

from PIL import Image
import os
import math

# Proporciones objetivo
TARGET_RATIOS = {
    '650x1024': 650/1024,
    '1024x1024': 1,
    '700x1024': 700/1024,
    '1024x450': 1024/450
}

def closest_ratio(image_path):
    """Encuentra la proporción objetivo más cercana a la imagen original"""
    with Image.open(image_path) as img:
        width, height = img.size
        original_ratio = width / height
        
        closest = None
        min_diff = float('inf')
        
        for ratio_name, target_ratio in TARGET_RATIOS.items():
            diff = abs(math.log(original_ratio) - math.log(target_ratio))
            if diff < min_diff:
                min_diff = diff
                closest = ratio_name
                
        return closest

def resize_image(image_path, output_folder):
    """Redimensiona la imagen a la proporción más cercana"""
    target_name = closest_ratio(image_path)
    target_width, target_height = map(int, target_name.split('x'))
    
    with Image.open(image_path) as img:
        # Convertir RGBA a RGB si es necesario
        if img.mode == 'RGBA':
            img = img.convert('RGB')
        
        img_ratio = img.width / img.height
        target_ratio = target_width / target_height
        
        if img_ratio > target_ratio:
            new_width = target_width
            new_height = int(target_width / img_ratio)
        else:
            new_height = target_height
            new_width = int(target_height * img_ratio)
        
        resized_img = img.resize((new_width, new_height), Image.LANCZOS)
        
        filename = os.path.basename(image_path)
        name, ext = os.path.splitext(filename)
        
        # Forzar extensión .jpg para JPEG
        output_path = os.path.join(output_folder, f"{name}_{target_name}.jpg")
        resized_img.save(output_path, quality=95)
        
        print(f"Imagen {filename} redimensionada a {target_name} ({new_width}x{new_height})")

def process_folder(input_folder, output_folder):
    """Procesa todas las imágenes en la carpeta de entrada"""
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    for filename in os.listdir(input_folder):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.webp')):
            image_path = os.path.join(input_folder, filename)
            try:
                resize_image(image_path, output_folder)
            except Exception as e:
                print(f"Error procesando {filename}: {str(e)}")
# Uso
input_folder = "/home/jfisherr/Escritorio/cuadros"  # Cambia esto
output_folder = "/home/jfisherr/Escritorio/copia_cuadros"  # Cambia esto

process_folder(input_folder, output_folder)
# Importing Image class from PIL module
from PIL import Image

'''
def attach_images_new( num_pieces, total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces):
		x_offset = 0
		for j in range(num_pieces):
			image_cut = Image.open("im_mill/2019-03-05-21-01-02_"+str(j*64)+"_"+str(i*64)+"_synthesized_image.jpg")
			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	# new_im.show()
	# new_im.save("new_im_"+str(i)+"_"+str(j)+".png")
	new_im.save("new_map1.png")

def attach_images_old( num_pieces, total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces):
		x_offset = 0
		for j in range(num_pieces):
			image_cut = Image.open("im_mill/2019-03-05-21-01-02_"+str(j*64)+"_"+str(i*64)+"_input_label.jpg")
			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	#new_im.show()
	new_im.save("old_map1.png")
'''

def attach_images( num_pieces_width, num_pieces_height,total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces_height):
		x_offset = 0
		for j in range(num_pieces_width):
			image_cut = Image.open("images_cut/img_cut_"+str(i)+"_"+str(j)+".png")
			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	# new_im.show()
	# new_im.save("new_im_"+str(i)+"_"+str(j)+".png")
	new_im.save("mappa.png")

def attach_images_new( num_pieces_width, num_pieces_height, total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces_height):
		x_offset = 0
		for j in range(num_pieces_width):
			image_cut = Image.open("images_millimap/img_cut_"+str(i)+"_"+str(j)+"_synthesized_image.jpg")
			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	# new_im.show()
	# new_im.save("new_im_"+str(i)+"_"+str(j)+".png")
	new_im.save("new_map.png")

def attach_images_new_final( num_pieces_width, num_pieces_height, total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces_height):
		x_offset = 0
		for j in range(num_pieces_width):

			try:
				image_cut = Image.open("images_millimap/img_cut_"+str(i)+"_"+str(j)+"_synthesized_image.jpg")
			except:
				image_cut = Image.open("images_millimap/img_cut_"+str(i)+"_"+str(j)+"_input_label.jpg")

			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	# new_im.show()
	# new_im.save("new_im_"+str(i)+"_"+str(j)+".png")
	new_im.save("final_map.png")

def attach_images_old( num_pieces_width, num_pieces_height, total_width, total_height):
	
	new_im = Image.new('RGB', (total_width, total_height))

	y_offset=0
	for i in range(num_pieces_height):
		x_offset = 0
		for j in range(num_pieces_width):
			image_cut = Image.open("images_millimap/img_cut_"+str(i)+"_"+str(j)+"_input_label.jpg")
			new_im.paste(image_cut, (x_offset,y_offset))
			x_offset += image_cut.size[0]
		y_offset += image_cut.size[1]

	#new_im.show()
	new_im.save("old_map.png")

def cut_image(image, num_pieces_width, num_pieces_height, resolution):
	# Setting the points for cropped image
	for i in range(num_pieces_height):
		for j in range(num_pieces_width):
			left = resolution*j
			top = resolution*i
			right = resolution*(j+1)
			bottom = resolution*(i+1)
			im1 = image.crop((left, top, right, bottom))

			# Cropped image of above dimension
			# (It will not change original image)
			 
			# Shows the image in image viewer
			#im1.show()	
			im1.save("images_cut/img_cut_"+str(i)+"_"+str(j)+".png")
 
# Opens a image in RGB mode

im = Image.open(r"mappa_3.pgm")

# Size of the image in pixels (size of original image)
# (This is not mandatory)
#width  = im.size[0]
#height = im.size[1]
#print(im.size)
#resolution = 64


resolution = 256
width= resolution*8
height = resolution*5

num_pieces_width = int(width/resolution)
num_pieces_height = int(height/resolution)
print(num_pieces_width, " ", num_pieces_height)

#cut_image(im, num_pieces_width, num_pieces_height, resolution)
#attach_images(num_pieces_width, num_pieces_height, width, height)

#implementare algoritmo di machine learning di millimap

attach_images_old(num_pieces_width, num_pieces_height, width, height)
#attach_images_new(num_pieces_width, num_pieces_height, width, height)
attach_images_new_final(num_pieces_width, num_pieces_height, width, height)


 
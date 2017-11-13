# For CPU
pip install tensorflow
# For GPU
pip install tensorflow-gpu
sudo apt-get update
sudo apt-get install protobuf-compiler python-pil python-lxml
sudo pip install jupyter
sudo pip install matplotlib
git clone https://github.com/tensorflow/models.git
cd models/research
echo "export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim" >> ~/.bashrc
sudo python setup.py build
sudo python setup.py install
python object_detection/builders/model_builder_test.py

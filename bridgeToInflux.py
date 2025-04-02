import influxdb_client, os, time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

import os
import sys
import subsprocess

token = "ODOAfu91GCSOA7vpeNLMKsZGtZQQsMM2mI9YHIhxm_WmX4xLFTJzOADtY3C-nbysbaDtQVdOohpYkQjeMPKFcA=="
org = "FiFr"
url = "http://127.0.0.1:8086"

# Fichier source
FICHIER_SOURCE = "/home/ludfr/MonSandPanda.txt"


def getRAMofPID(pid):
    process = subprocess.run(
        "pidstat -p "+pid+" -r | grep "+pid+" | awk '{print($6)}'",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, timeout=120)
    ram_as_str = process.stdout.decode('utf-8', errors='ignore')
    return int(ram_as_str)

def lire_et_encoder(nom, niveau_actuel, niveau_max, ram):
    """Lit le fichier, enregistre les données dans InfluxDB et efface le contenu du fichier."""
    try:
        # Connexion à InfluxDB
        client = influxdb_client.InfluxDBClient(url=url, token=token, org=org)
        bucket="fanstein"
        write_api = client.write_api(write_options=SYNCHRONOUS)
        point = (
        Point("sandPanda")
        .tag("id", nom)
        .tag("host", "atoem")
        .field("now", (float)(niveau_actuel))
        .field("relative", ((float)(niveau_actuel)/(float)(niveau_max)))
        .field("ram", ram)
        )
        write_api.write(bucket=bucket, org="FiFr", record=point)
        #print(f"Donnée insérée: {nom, niveau_actuel, niveau_max}")

        # Fermer la connexion

        client.close()

        #print("Données insérées")

    except Exception as e:
        print(f"Erreur: {e}")

if __name__ == "__main__":
    lire_et_encoder(sys.argv[1], sys.argv[2], sys.argv[3], getRAMofPID(sys.argv[4]))

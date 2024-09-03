# simulation

# Azure repository
To speed up the process of deployment and testing of the system, we have stored the docker image on the azure registery. 

In order to enable the access to the registry, you need to login the azure accounts.

Here's a simple guide,

Intall azure-cli:

```
curl -sL https://aka.ms/InstallAzureCLIDeb | sudo bash
```

Login with your browser and company account

```
az login
az acr login --name leapaidevcr
```

All done!

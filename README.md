### AWS Download Instructions
1. [Create an AWS account (optional)](https://aws.amazon.com/premiumsupport/knowledge-center/create-and-activate-aws-account/)
2. [Install the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2.html)
3. Create a `root` folder to store the dataset, example: `/path/to/data/canoe/`. Each sequence will then be a folder under `root`.
4. Use the AWS CLI to download either the entire dataset or only the desired sequences and sensors. Add `--no-sign-request` after each of the following commands if you're not going to use an AWS account. For example, the following command will download the entire canoe dataset: 

```bash
root=/path/to/data/canoe/
aws s3 sync s3://canoe-data/ $root
```

The following command will list all the top-level prefixes (sequences):

```bash
root=/path/to/data/canoe/
aws s3 ls s3://canoe-data
```
Alternatively, [our website](canoe.cs.toronto.edu/#/download) can be used to browse through sequences as well as pick and choose what data to download. 
The website will then generate a list of AWS CLI commands that can be run as a bash script. 
These commands will look something like:

```bash
root=/path/to/data/canoe/
cd $root
aws s3 sync s3://canoe-data/canoe-2025-08-21-19-16 ./canoe-2025-08-21-19-16
```
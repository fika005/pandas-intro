import pandas as pd
import matplotlib.pyplot as plt


def read_data(file_name):
    col_names = ["class", "age", "menopause", "tumor-size", "inv-nodes",
                 "node-caps", "deg-malig", "breast", "breast-quad", "irradiat"]
    return pd.read_csv(file_name, names=col_names)


def most_common_class(data):
    return data["class"].value_counts().index[0]


def most_common_age_menopause(data):
    recurrence_data = data[data["class"] == "recurrence-events"]
    return recurrence_data["age"].value_counts().index[0], recurrence_data["menopause"].value_counts().index[0]


def plot_recurrence_age(data):
    recurrence_data = data[data["class"] == "recurrence-events"]
    recurrence_data.groupby("age")["class"].count().plot()
    plt.ylabel('count')
    plt.show()


if __name__ == "__main__":
    data = read_data("breast-cancer.data")
    print("most common classification:", most_common_class(data))
    age, menopause = most_common_age_menopause(data)
    print("most common age for recurrence:", age)
    print("most common menopause for recurrence:", menopause)
    plot_recurrence_age(data)

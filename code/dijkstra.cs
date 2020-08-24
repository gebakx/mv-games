using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

public class dijkstra : MonoBehaviour
{
    HashSet<string> vertices = new HashSet<string> {"a", "b", "c", "d", "e", "f"};

    Dictionary<string,Dictionary<string,int>> edges = new Dictionary<string,Dictionary<string,int>>
        { 
            { "a", new Dictionary<string,int> { {"b", 4}, {"c", 2} } },
            { "b", new Dictionary<string,int> { {"c", 5}, {"d", 10} } },
            { "c", new Dictionary<string,int> { {"e", 3} } },
            { "d", new Dictionary<string,int> { {"f", 11} } },
            { "e", new Dictionary<string,int> { {"d", 4} } },
            { "f", new Dictionary<string,int> { } }
        };

    Dictionary<string, int> h = new Dictionary<string, int> 
        { {"a", 20}, {"b", 15}, {"c", 12}, {"d", 10}, {"e", 9}, {"f", 0} };

    void Start()
    {
        Show(Dijkstra("a", "f"));
    }

    List<string> Dijkstra(string source, string target) 
    {
        var d = vertices.ToDictionary(g => g, g => int.MaxValue);
        var prev = vertices.ToDictionary(g => g, g => " ");
        d[source] = 0;
        HashSet<String> Q = vertices;
        while (Q.Count() > 0)
        {
            string v = Q.Select(x => (d[x], x)).Min().Item2;
            Q.Remove(v);
            foreach (var pair in edges[v])
            {
                int alt = d[v] + pair.Value;
                if (alt < d[pair.Key]) 
                {
                    d[pair.Key] = alt;
                    prev[pair.Key] = v;
                }
            }   
        }
        List<string> path = new List<string>();
        path.Insert(0,target);
        while (prev[target] != " ")
        {
            target = prev[target];
            path.Insert(0,target);
        }
        return path;
    }

    void Show(List<string> l)
    {
        string s = "Path:\n";
        foreach (var x in l)
            s += " " + x;
        Debug.Log(s);        
    }
}
